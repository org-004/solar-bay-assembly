// See src/layout_plan.h: the planning side (MoveWorld and PlanLayout).
//
// PlanLayout works in three stages. It assembles every arm's row of blocks
// from the checker's steps and poses (one block for each step of the arm:
// its move, approach, work and retreat, each with its duration and its
// path), plans every move alone in its world on several threads, and then
// places the blocks on one clock, one at a time and in a fixed order, so
// that nothing in the plan depends on the number of threads or on time.

#include "src/layout_plan.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <thread>
#include <tuple>

#include "src/layout_bays.h"
#include "src/layout_clear_out.h"
#include "src/layout_plan_stages.h"
#include "src/layout_stages.h"
#include "src/layout_torque.h"
#include "src/portable_random.h"
#include "src/rrt.h"

namespace cell::layout {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
// How far a block is put off when another arm is in its way, and how often.
constexpr double kShiftS = 0.25;
constexpr int kShiftTries = 2000;

std::string Fmt(const char* format, ...) {
  char buffer[2048];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Config ToConfig(const std::array<double, 6>& deg, double track) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = deg[j] * kDeg;
  c[6] = track;
  return c;
}

Config WaitPose(const SceneArm& sa) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = sa.wait[j];
  c[6] = sa.wait_track;
  return c;
}

ArmPose ToPose(const Config& c) {
  ArmPose p;
  for (int j = 0; j < 6; ++j) p.q[j] = c[j];
  p.track = c[6];
  return p;
}

bool SameConfig(const Config& a, const Config& b) {
  for (int j = 0; j < 7; ++j) {
    if (std::abs(a[j] - b[j]) > 1e-9) return false;
  }
  return true;
}

std::string ConfigText(const Config& c) {
  return Fmt("(%.1f, %.1f, %.1f, %.1f, %.1f, %.1f deg; track %.3f m)",
             c[0] / kDeg, c[1] / kDeg, c[2] / kDeg, c[3] / kDeg, c[4] / kDeg,
             c[5] / kDeg, c[6]);
}

// The union of a work point's one or two states of the bay.
Built Union(const std::vector<Built>& states) {
  std::set<int> rails, modules;
  Built b;
  for (const Built& s : states) {
    rails.insert(s.rails.begin(), s.rails.end());
    modules.insert(s.modules.begin(), s.modules.end());
    b.on_pallets = std::max(b.on_pallets, s.on_pallets);
  }
  b.rails.assign(rails.begin(), rails.end());
  b.modules.assign(modules.begin(), modules.end());
  return b;
}

bool IsHoleKind(const std::string& kind) {
  return kind == "locate" || kind == "pin" || kind == "swage";
}

// One move to plan.
struct MoveJob {
  int arm = -1;
  int block = -1;       // Index into the list of blocks.
  int segment = -1;     // Index of the move in the block's segments.
  int step = 0;         // The step's number.
  Config from{}, to{};
  std::string tool, held;
  Built built;
  std::string from_label, to_label;
  std::uint32_t seed = 0;
  bool in_range = true;
  int stage = 0;        // Planned in this stage's scene (staged work).
  // Staged work: the bay without the modules set after the hole's module,
  // tried when the move is not found in `built` (PlanNarrowed).
  bool narrowable = false, narrowed = false;
  Built narrow;
  // Results.
  bool found = false;
  int budget = 0;
  std::int64_t queries = 0;
  std::vector<Config> path;
  double planned_s = 0.0;
  double model_s = 0.0;
  std::string note;
  std::vector<std::string> log;  // Each budget's RRT report.
};

// One step of one arm: its segments, with durations, from time 0.
struct Block {
  int arm = -1;
  int step_index = -1;  // Into report.steps.
  int number = 0;       // The step's number.
  std::vector<Segment> segs;
  std::vector<int> moves;  // Indices into the moves, in segment order.
  bool in_range = true;
  bool parking = false;  // The move to the wait pose before the step.
  bool magazine = false; // Staged: that move follows a magazine take.
  int stage = 0;         // Staged work: its step's stage.
  int after = -1;        // A block it gave way to, placed before it.
  // Placed.
  bool placed = false;
  double start = 0.0, end = 0.0, work_end = 0.0;
};

// What must have happened before a block may start.
struct Need {
  int step = 0;          // The step's number.
  bool work_end = false; // The end of its work segment, else of the step.
};

struct ArmEnd {
  Config config{};
  std::string tool, held;
};

}  // namespace

// ------------------------------------------------------------- MoveWorld

namespace {

PairQuery MoveQuery(const LayoutScene& scene, int arm, const std::string& tool,
                    const std::string& held, const Built& built) {
  // The other arms where the data has them (at their wait poses, with
  // their first tools), so that no move passes through an arm that stands
  // at its wait pose.
  ObstacleOptions options;
  options.other_arms = true;
  return PairQuery(scene, arm, scene.Moving(arm, tool, held),
                   scene.Obstacles(arm, tool, built, DesignPairs{}, options));
}

}  // namespace

MoveWorld::MoveWorld(const LayoutScene& scene, int arm, const std::string& tool,
                     const std::string& held, const Built& built,
                     double clearance_m)
    : scene_(scene),
      arm_(arm),
      clearance_m_(clearance_m),
      query_(MoveQuery(scene, arm, tool, held, built)) {}

Nearest MoveWorld::Least(mjData* d, const Config& c, double far) const {
  scene_.SetPose(d, arm_, ToPose(c));
  query_.Place(d);
  return query_.Least(d, far);
}

bool MoveWorld::Free(mjData* d, const Config& c) const {
  scene_.SetPose(d, arm_, ToPose(c));
  query_.Place(d);
  return !query_.Closer(d, clearance_m_);
}

// ------------------------------------------------------------ PlanLayout

namespace {

class Planner {
 public:
  // Plans bay `bay` of several (src/layout_bays.h) with the checker's
  // `report` when one is given, the bay lying on the joint clock as `clock`
  // says.
  Planner(const Layout& layout, const ModelPaths& paths,
          const std::string& sha, const PlanOptions& options, int bay = 0,
          const CheckReport* report = nullptr, const BayClock& clock = {})
      : layout_(layout), paths_(paths), sha_(sha), options_(options),
        bay_(bay), given_(report), clock_(clock), shift_(clock.shift) {}

  PlanResult Run(bool plan = true, bool verify = true);
  std::vector<std::string> Why(const std::vector<int>& steps);

 private:
  bool InRange(int number) const {
    if (options_.first_step == 0 && options_.last_step == 0) return true;
    return number >= options_.first_step && number <= options_.last_step;
  }
  bool Partial() const {
    return options_.first_step != 0 || options_.last_step != 0;
  }
  void Fault(const std::string& s) { result_.faults.push_back(s); }

  bool Assemble();
  void PlanMoves();
  void PlanOne(MoveJob& job, mjData* d, const std::vector<int>& budgets);
  void PlanNarrowed(MoveJob& job, mjData* d);
  bool Schedule();
  bool ScheduleSlowed();
  void ResetSchedule();
  void FillPlan();

  // The schedule's pieces.
  struct ArmState {
    double free = 0.0;
    size_t next = 0;          // Index into the arm's blocks.
    ArmEnd end;
  };
  struct Conflict {
    bool any = false;
    int other = -1;
    double t = 0.0;
    Nearest nearest;
    bool standing = false;    // The other arm stands for good there.
    bool tube = false;        // The tube coming in, not another arm.
    Config mine{}, theirs{};
    int other_segment = -1;
    std::string my_kind;
  };
  // Where arm `a` is at time t of its placed row.
  void StateAt(int a, double t, Config* c, std::string* tool,
               std::string* held, int* segment) const;
  Conflict Check(int a, const Block& b, double s, mjData* d);
  const std::vector<MovingGeom>& ArmSet(int a, const std::string& tool,
                                        const std::string& held);
  const std::vector<Obstacle>& ArmObstacles(int a, const std::string& tool,
                                            const std::string& held);
  // Staged work (src/layout_plan_stages.h): when the transfer to stage s
  // ends, -1 while a block of an earlier stage is not placed; the leaders'
  // parts of a move of `phase` at `stage`.
  double TransferEnd(int s) const;
  void Lead(int stage, const std::string& phase, Built* b) const {
    LeadersParts(staged_ ? &scopes_[stage] : nullptr, phase, b);
  }
  void Place(int a, Block& b, double s, double ready,
             const std::string& need_reason, const std::string& wait_reason);
  std::string SegmentText(int a, int segment) const;

  const Layout& layout_;
  const ModelPaths& paths_;
  const std::string sha_;
  const PlanOptions options_;
  const int bay_;
  const CheckReport* given_;
  const BayClock clock_;
  const double shift_;
  PlanResult result_;
  CheckReport report_;
  std::vector<std::unique_ptr<LayoutScene>> scenes_;  // One for each stage.
  LayoutScene* scene_ = nullptr;                      // The first stage's.
  bool staged_ = false;
  std::vector<StageScope> scopes_;
  std::map<int, int> stage_of_step_;                  // Step number to stage.
  std::vector<int> staged_pos_;                       // Module to its place in the order.
  PlanFigures f_;
  std::vector<ArmMotion> motions_;
  // The layout's torque_limit_scale (src/layout_torque.h): the module arm's
  // dynamics, and its name; null without the field.
  std::unique_ptr<ArmDynamics> dynamics_;
  std::string torque_arm_;
  std::vector<MoveJob> moves_;
  std::vector<Block> blocks_;
  std::vector<std::vector<int>> arm_blocks_;   // Per arm, block indices.
  std::map<int, int> block_of_step_;           // Step number to block.
  std::map<int, int> index_of_step_;           // Step number to report.steps index.
  std::vector<ArmPlan> rows_;
  std::vector<ArmState> state_;
  std::map<std::tuple<int, std::string, std::string>, std::vector<MovingGeom>>
      sets_;
  std::map<std::tuple<int, std::string, std::string>, std::vector<Obstacle>>
      obstacle_sets_;
  std::vector<BayEvent> events_;
  // The blocks in the order they were placed, for each its arm's row
  // length and state before it, and the pairs (block taken back, block it
  // gave way to) so far.
  std::vector<int> placed_order_;
  std::map<int, std::pair<size_t, ArmState>> undo_;
  std::set<std::pair<int, int>> yielded_;
  double clamp_end_ = 0.0;
  bool all_found_ = true;
  bool scheduled_ = false;
  bool prepared_ = false;
  std::int64_t schedule_queries_ = 0;
  // The torque bound (ScheduleSlowed): each block's segments as timed
  // before any move was slowed, whether a move was, and, when not empty,
  // each block's place in the order of placing that Schedule follows in
  // place of the earliest start.
  std::vector<std::vector<Segment>> plain_segs_;
  bool slowed_ = false;
  std::vector<int> rank_;
  std::vector<double> release_;  // Each block's earliest start, when not empty.
};

const SolvedPoint* FindPoint(const CheckReport& r, const std::string& kind,
                             int index) {
  for (const SolvedPoint& p : r.points) {
    if (p.kind == kind && p.index == index) return &p;
  }
  return nullptr;
}

bool Planner::Assemble() {
  const int n = scene_->ArmCount();
  arm_blocks_.assign(n, {});
  const std::vector<std::string> leaders =
      ArmsDoing(layout_, {"rail_take", "rail_set", "module_take", "module_set"});
  std::vector<WorkPoint> all_points;
  std::map<std::string, std::vector<WorkPoint>> points;
  for (const std::string& kind : WorkKinds()) {
    points[kind] = WorkPoints(layout_, kind);
  }
  auto work_point = [&](const std::string& kind, int index) -> const WorkPoint* {
    auto it = points.find(kind);
    if (it == points.end()) return nullptr;
    for (const WorkPoint& p : it->second) {
      if (p.index == index) return &p;
    }
    return nullptr;
  };
  struct Now {
    Config at{};
    std::string tool, held, label = "wait";
    const SolvedPoint* last_point = nullptr;  // Of its last working step.
    const WorkPoint* last_wp = nullptr;       // Its bay 0's point.
    int last_block = -1;
    bool last_retreat = false;  // The last block ends with a retreat.
    std::string last_phase, last_kind;  // Of its last step.
    int last_stage = 0;
  };
  std::vector<Now> now(n);
  for (int a = 0; a < n; ++a) {
    now[a].at = WaitPose(scene_->arm(a));
    now[a].tool = scene_->arm(a).use->first_tool;
  }
  const int pallet_total = PointCount(layout_, "module_take");
  const int per_bay = ModulesPerBay(report_);
  bool ok = true;
  // The bay after the arm's previous step, for a tool change or a return.
  auto bay_after = [&](int a, const std::string& phase, int stage) {
    Built b;
    b.on_pallets = pallet_total;
    if (now[a].last_wp != nullptr) {
      StepScene at = scenes_[now[a].last_stage]->AtWorkPoint(*now[a].last_wp, Leg::kRetreat);
      if (staged_) BuiltInStage(scopes_[now[a].last_stage], pallet_total, &at);
      b = Union(at.built);
    }
    if (staged_ && (now[a].last_wp == nullptr || now[a].last_stage < stage)) {
      b = BuiltAtStageStart(scopes_[stage], pallet_total);
    }
    const bool leader = std::find(leaders.begin(), leaders.end(),
                                  scene_->arm(a).use->name) != leaders.end();
    if (!leader) Lead(stage, phase, &b);
    return BuiltInBay(b, bay_, per_bay);
  };
  auto add_move = [&](int a, Block& block, int block_index, const Config& to,
                      const std::string& to_label, const Built& built,
                      double& t) {
    if (SameConfig(now[a].at, to)) return;
    MoveJob job;
    job.arm = a;
    job.block = block_index;
    job.segment = static_cast<int>(block.segs.size());
    job.step = block.number;
    job.from = now[a].at;
    job.to = to;
    job.tool = now[a].tool;
    job.held = now[a].held;
    job.built = built;
    job.from_label = now[a].label;
    job.to_label = to_label;
    job.in_range = block.in_range;
    job.stage = block.stage;
    Segment s;
    s.kind = "move";
    s.step = block.number;
    s.tool = job.tool;
    s.held = job.held;
    s.from = job.from_label;
    s.to = to_label;
    s.built = built;
    s.path = {job.from, job.to};
    s.t0 = t;
    s.t1 = t;  // Set when the move is planned.
    block.segs.push_back(s);
    block.moves.push_back(static_cast<int>(moves_.size()));
    moves_.push_back(job);
    now[a].at = to;
    now[a].label = to_label;
  };
  for (size_t si = 0; si < report_.steps.size(); ++si) {
    const BayStep& st = report_.steps[si];
    index_of_step_[st.number] = static_cast<int>(si);
    const int stage = StageIndexOf(layout_, st);
    stage_of_step_[st.number] = stage;
    if (st.arm.empty()) continue;
    const int a = scene_->ArmIndex(st.arm);
    if (a < 0) {
      Fault(Fmt("step %d: no arm %s", st.number, st.arm.c_str()));
      ok = false;
      continue;
    }
    const bool leader =
        std::find(leaders.begin(), leaders.end(), st.arm) != leaders.end();
    // Parking: after its last step of the rail phase an arm that follows
    // another goes to its wait pose. The move belongs to the arm's next
    // step and is a block of its own, which starts as soon as the arm is
    // free; the next step then starts from the wait pose when it may. In
    // staged work an arm parks after its magazine take as well.
    const bool magazine = staged_ && now[a].last_kind == "pins_take";
    if ((!leader && now[a].last_phase == "rails" && st.phase != "rails" &&
         st.kind != "to_wait") || magazine) {
      Block park;
      park.arm = a;
      park.step_index = static_cast<int>(si);
      park.number = st.number;
      park.in_range = InRange(st.number);
      park.parking = true;
      park.magazine = magazine;
      park.stage = stage;
      const Config w = WaitPose(scene_->arm(a));
      if (!now[a].held.empty() && !magazine) {  // A magazine has no geoms.
        Fault(Fmt("step %d (%s): %s still holds %s when it parks", st.number,
                  st.what.c_str(), st.arm.c_str(), now[a].held.c_str()));
        ok = false;
      }
      double tp = 0.0;
      const int pi = static_cast<int>(blocks_.size());
      add_move(a, park, pi, w, "wait", bay_after(a, magazine ? "rails" : st.phase, stage), tp);
      if (!park.segs.empty()) {
        blocks_.push_back(park);
        arm_blocks_[a].push_back(pi);
        now[a].last_retreat = false;
        now[a].last_block = pi;
      }
    }
    now[a].last_phase = st.phase;
    now[a].last_kind = st.kind;
    Block block;
    block.arm = a;
    block.step_index = static_cast<int>(si);
    block.number = st.number;
    block.in_range = InRange(st.number);
    block.stage = stage;
    const int bi = static_cast<int>(blocks_.size());
    double t = 0.0;
    auto push = [&](const std::string& kind, const std::string& tool,
                    const std::string& held, std::vector<Config> path,
                    double duration) {
      Segment s;
      s.kind = kind;
      s.step = st.number;
      s.tool = tool;
      s.held = held;
      s.path = std::move(path);
      s.t0 = t;
      s.t1 = t + duration;
      t = s.t1;
      block.segs.push_back(s);
    };
    if (st.kind == "tool_change" || st.kind == "to_wait") {
      Config target{};
      std::string label;
      if (st.kind == "tool_change") {
        const SolvedToolChange* tc = nullptr;
        for (const SolvedToolChange& c : report_.tool_changes) {
          if (c.arm == st.arm) tc = &c;
        }
        if (tc == nullptr || !tc->reached) {
          Fault(Fmt("step %d (%s): %s has no tool-change pose", st.number,
                    st.what.c_str(), st.arm.c_str()));
          ok = false;
          continue;
        }
        target = ToConfig(tc->deg, now[a].at[6]);
        label = "tools";
      } else {
        target = WaitPose(scene_->arm(a));
        label = "wait";
      }
      if (!now[a].held.empty()) {
        Fault(Fmt("step %d (%s): %s still holds %s", st.number,
                  st.what.c_str(), st.arm.c_str(), now[a].held.c_str()));
        ok = false;
      }
      add_move(a, block, bi, target, label, bay_after(a, st.phase, stage), t);
      if (st.kind == "tool_change") {
        push("work", now[a].tool, now[a].held, {target},
             scene_->arm(a).use->tool_change_s);
        now[a].tool = st.tool_after;
        // After the last tool change of the bay the arm goes back to its
        // wait pose, so that it stands there when the bay goes out.
        bool later = false;
        for (size_t k = si + 1; k < report_.steps.size(); ++k) {
          if (report_.steps[k].arm == st.arm) later = true;
        }
        if (!later) {
          add_move(a, block, bi, WaitPose(scene_->arm(a)), "wait",
                   bay_after(a, st.phase, stage), t);
        }
      }
      if (block.segs.empty()) continue;  // Already there.
      now[a].last_retreat = false;
      now[a].last_block = bi;
      blocks_.push_back(block);
      arm_blocks_[a].push_back(bi);
      block_of_step_[st.number] = bi;
      continue;
    }
    // A working step: its poses at the bay's point, its bay as bay 0's.
    const int point = PointIndexInBay(st, bay_, per_bay);
    const SolvedPoint* p = FindPoint(report_, st.kind, point);
    const WorkPoint* wp = work_point(st.kind, st.index);
    const Work* work = FindWork(layout_, st.kind, point);
    if (p == nullptr || wp == nullptr || work == nullptr || !p->work_reached ||
        p->approach_leg_deg.empty() || p->retreat_leg_deg.empty()) {
      Fault(Fmt("step %d (%s): no work pose or legs for %s %d", st.number,
                st.what.c_str(), st.kind.c_str(), point));
      ok = false;
      continue;
    }
    if (p->tool != now[a].tool) {
      Fault(Fmt("step %d (%s): %s has %s on its flange, the point needs %s",
                st.number, st.what.c_str(), st.arm.c_str(),
                now[a].tool.c_str(), p->tool.c_str()));
      ok = false;
    }
    const Config work_c = ToConfig(p->work_deg, p->track);
    std::vector<Config> approach;
    for (auto it = p->approach_leg_deg.rbegin();
         it != p->approach_leg_deg.rend(); ++it) {
      approach.push_back(ToConfig(*it, p->track));
    }
    std::vector<Config> retreat;
    for (const auto& q : p->retreat_leg_deg) {
      retreat.push_back(ToConfig(q, p->track));
    }
    const ArmMotion& m = motions_[a];
    const double ta = LegDuration(approach, p->approach_length_m, m, f_);
    const double tr = RetreatDuration(retreat, p->retreat_length_m, m, f_);
    if (ta < 0.0 || tr < 0.0) {
      Fault(Fmt("step %d (%s): a leg cannot be timed", st.number,
                st.what.c_str()));
      ok = false;
    }
    // Two steps at the same work pose: the previous block loses its
    // retreat and this one has no move or approach.
    if (now[a].last_block >= 0 && now[a].last_retreat &&
        now[a].last_point != nullptr &&
        SameConfig(ToConfig(now[a].last_point->work_deg,
                            now[a].last_point->track),
                   work_c)) {
      Block& prev = blocks_[now[a].last_block];
      prev.segs.pop_back();
      now[a].at = work_c;
      now[a].held = now[a].last_point->held_work;
      now[a].label = now[a].last_point->item + " work";
      if (now[a].held != p->held_work) {
        Fault(Fmt("step %d (%s): %s holds %s, the work needs %s", st.number,
                  st.what.c_str(), st.arm.c_str(), now[a].held.c_str(),
                  p->held_work.c_str()));
        ok = false;
      }
    } else {
      // A pin magazine has no geoms: an arm that took one goes to its
      // holes with it, though the holes' legs name no part.
      if (now[a].held != p->held_approach &&
          !(now[a].held == "pin magazine" && p->held_approach.empty())) {
        Fault(Fmt("step %d (%s): %s holds \"%s\" before the approach, which "
                  "needs \"%s\"",
                  st.number, st.what.c_str(), st.arm.c_str(),
                  now[a].held.c_str(), p->held_approach.c_str()));
        ok = false;
      }
      // Staged work: the bay kept to the stage, and a magazine taken
      // before the stage's modules are set.
      StepScene at = scenes_[stage]->AtWorkPoint(*wp, Leg::kApproach);
      if (staged_) BuiltInStage(scopes_[stage], pallet_total, &at);
      Built built = Union(at.built);
      if (!leader) Lead(stage, staged_ && st.kind == "pins_take" ? "rails" : st.phase, &built);
      Built narrow = built;  // Without the modules set after the hole's.
      narrow.modules.clear();
      for (int m : built.modules) {
        if (staged_ && staged_pos_[m] <= staged_pos_[std::clamp(st.index / 4, 0, 8)]) {
          narrow.modules.push_back(m);
        }
      }
      const size_t before = moves_.size();
      add_move(a, block, bi, approach.front(), p->item + " approach",
               BuiltInBay(built, bay_, per_bay), t);
      if (moves_.size() > before && staged_ && !leader && IsHoleKind(st.kind) &&
          narrow.modules != built.modules) {
        moves_.back().narrowable = true;
        moves_.back().narrow = BuiltInBay(narrow, bay_, per_bay);
      }
      push("approach", p->tool, p->held_approach, approach, ta);
    }
    push("work", p->tool, p->held_approach, {work_c}, work->process_s);
    push("retreat", p->tool, p->held_retreat, retreat, tr);
    now[a].at = retreat.back();
    now[a].held = p->held_retreat;
    now[a].label = p->item + " retreat";
    now[a].last_point = p;
    now[a].last_wp = wp;
    now[a].last_stage = stage;
    now[a].last_block = bi;
    now[a].last_retreat = block.segs.back().kind == "retreat";
    blocks_.push_back(block);
    arm_blocks_[a].push_back(bi);
    block_of_step_[st.number] = bi;
  }
  // Every arm ends at its wait pose with its first tool.
  for (int a = 0; a < n; ++a) {
    const SceneArm& sa = scene_->arm(a);
    if (!SameConfig(now[a].at, WaitPose(sa)) || now[a].tool != sa.use->first_tool) {
      Fault(Fmt("%s does not end the bay at its wait pose with %s",
                sa.use->name.c_str(), sa.use->first_tool.c_str()));
      ok = false;
    }
  }
  // The seeds: one generator, one draw per move, in the steps' order.
  std::mt19937 rng(f_.seed);
  for (MoveJob& job : moves_) {
    job.seed = static_cast<std::uint32_t>(UniformIndex(rng, 0x7fffffffu)) + 1u;
  }
  return ok;
}

void Planner::PlanOne(MoveJob& job, mjData* d, const std::vector<int>& budgets) {
  const auto begun = std::chrono::steady_clock::now();
  struct Report {
    const MoveJob& job;
    std::chrono::steady_clock::time_point begun;
    ~Report() {
      if (std::getenv("PLAN_DEBUG") == nullptr) return;
      std::fprintf(stderr, "cost step %d %s to %s: %lld queries, %.2f s, %s\n",
                   job.step, job.from_label.c_str(), job.to_label.c_str(),
                   static_cast<long long>(job.queries),
                   std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - begun)
                       .count(),
                   job.found ? "found" : job.note.c_str());
    }
  } report{job, begun};
  // The other arms (their waiting tools ride their carriages) stand at
  // their wait poses, whatever moves this data was used for before: a
  // thread plans moves of several arms, and the plan must not depend on
  // which.
  const LayoutScene& sc = *scenes_[job.stage];
  sc.AllAtWait(d);
  job.log.clear();
  const double clear = f_.clearance_m + f_.plan_pad_m;
  const MoveWorld world(sc, job.arm, job.tool, job.held, job.built, clear);
  const ArmMotion& m = motions_[job.arm];
  std::int64_t queries = 0;
  // Start and goal: a finding when either is not free.
  for (int end = 0; end < 2; ++end) {
    const Config& c = end == 0 ? job.from : job.to;
    const Nearest nearest = world.Least(d, c, clear);
    ++queries;
    if (nearest.distance < clear) {
      job.note += Fmt("%s%s not free at %.4f m: %s at %.4f m",
                      job.note.empty() ? "" : "; ",
                      end == 0 ? "start" : "goal", clear,
                      sc.PairText(nearest).c_str(), nearest.distance);
    }
  }
  if (!job.note.empty()) {
    job.queries = queries;
    job.model_s =
        ModelMoveSeconds(sc, job.arm, ToPose(job.from), ToPose(job.to));
    return;
  }
  const bool fixed = std::abs(job.from[6] - job.to[6]) <= 1e-12;
  const int dims = fixed ? 6 : 7;
  const double track = job.from[6];
  auto to_config = [&](const Eigen::VectorXd& v) {
    Config c{};
    for (int j = 0; j < dims; ++j) c[j] = v[j];
    if (fixed) c[6] = track;
    return c;
  };
  const IsFree is_free = [&](const Eigen::VectorXd& v) {
    ++queries;
    return world.Free(d, to_config(v));
  };
  Eigen::VectorXd start(dims), goal(dims);
  for (int j = 0; j < dims; ++j) {
    start[j] = job.from[j];
    goal[j] = job.to[j];
  }
  RrtOptions ro;
  ro.try_direct_first = true;
  ro.edge_resolution = f_.edge_resolution;
  ro.weights = m.weights.head(dims);
  ro.max_seconds = 1e12;
  ro.seed = job.seed;
  const Eigen::VectorXd lower = m.lower.head(dims), upper = m.upper.head(dims);
  std::optional<JointPath> path;
  // The queries the checks of start and goal made above are counted; the
  // planner checks them once more and counts those too. The budgets are
  // tried in turn in one run (PlanRrtConnectBudgets); the queries of each
  // budget run out are counted as if it had been run on its own.
  std::vector<RrtReport> reports;
  path = PlanRrtConnectBudgets(start, goal, lower, upper, is_free, ro, budgets,
                               &reports);
  for (size_t k = 0; k < reports.size(); ++k) {
    const int budget = budgets[k];
    const RrtReport& rr = reports[k];
    if (k + 1 < reports.size()) queries += rr.collision_queries;
    job.log.push_back(Fmt(
        "budget %d in %d coordinates: %s, %d iterations, trees of %d and %d "
        "nodes, %lld queries",
        budget, dims, ToString(rr.status), rr.iterations, rr.start_tree_size,
        rr.goal_tree_size, static_cast<long long>(rr.collision_queries)));
    if (std::getenv("PLAN_DEBUG") != nullptr) {
      std::fprintf(stderr, "move step %d %s\n  from %s\n  to   %s\n", job.step,
                   job.log.back().c_str(), ConfigText(job.from).c_str(),
                   ConfigText(job.to).c_str());
    }
    if (k + 1 == reports.size() && path) {
      job.budget = rr.iterations == 0 ? 0 : budget;
      break;
    }
    if (rr.status != RrtStatus::kIterationLimit) {
      job.note = Fmt("planner: %s", ToString(rr.status));
      break;
    }
    job.note = Fmt("not found within %d iterations", budget);
  }
  job.model_s =
      ModelMoveSeconds(sc, job.arm, ToPose(job.from), ToPose(job.to));
  if (!path) {
    job.queries = queries;
    return;
  }
  job.note.clear();
  ShortenOptions so;
  so.max_iterations = f_.shorten_tries;
  so.max_seconds = 1e12;
  so.edge_resolution = f_.edge_resolution;
  so.weights = ro.weights;
  so.seed = job.seed;
  std::optional<JointPath> shortened = ShortenPath(*path, is_free, so);
  if (shortened) path = shortened;
  job.path.clear();
  for (const Eigen::VectorXd& v : *path) job.path.push_back(to_config(v));
  // The ends exactly as asked.
  job.path.front() = job.from;
  job.path.back() = job.to;
  job.queries = queries;
  // The layout's path_tries: the same search from more seeds, each path
  // shortened as the first, and the quickest timed path of all kept (the
  // first of equals, so the move's own path unless another is quicker). A
  // seed that finds nothing within the budgets adds nothing.
  if (f_.path_tries > 1) {
    std::vector<std::vector<Config>> paths = {job.path};
    std::vector<int> found_by = {job.budget};
    for (int k = 1; k < f_.path_tries; ++k) {
      RrtOptions rk = ro;
      rk.seed = job.seed + 0x9e3779b9u * static_cast<std::uint32_t>(k);
      std::vector<RrtReport> rk_reports;
      std::optional<JointPath> other = PlanRrtConnectBudgets(
          start, goal, lower, upper, is_free, rk, budgets, &rk_reports);
      if (!other) continue;
      ShortenOptions sk = so;
      sk.seed = rk.seed;
      if (std::optional<JointPath> s = ShortenPath(*other, is_free, sk)) other = s;
      std::vector<Config> cp;
      for (const Eigen::VectorXd& v : *other) cp.push_back(to_config(v));
      cp.front() = job.from;
      cp.back() = job.to;
      paths.push_back(std::move(cp));
      const RrtReport& last = rk_reports.back();
      found_by.push_back(last.iterations == 0
                             ? 0
                             : budgets[std::min(rk_reports.size(), budgets.size()) - 1]);
    }
    const int best = QuickestPath(paths, m, f_);
    if (best > 0) {
      job.path = paths[best];
      job.budget = found_by[best];
      job.log.push_back(Fmt("path try %d of %d kept, the quickest", best + 1,
                            f_.path_tries));
    }
    job.queries = queries;
  }
  job.planned_s = MoveDuration(job.path, m, f_);
  if (job.planned_s < 0.0) {
    job.note = "the path cannot be timed";
    return;
  }
  job.found = true;
}

// Staged work (src/layout_plan_stages.h): a move between the holes of module
// m not found where the next modules may have been set is planned without
// them; the next module's set then waits for it (Schedule).
// A narrowable move tries its own world without the staged fourth budget.
void Planner::PlanNarrowed(MoveJob& job, mjData* d) {
  std::vector<int> first = f_.budgets;
  if (job.narrowable && first.size() > 1) first.pop_back();
  PlanOne(job, d, job.narrowable ? first : f_.budgets);
  if (job.found || !job.narrowable) return;
  MoveJob narrow = job;
  narrow.built = job.narrow;
  narrow.note.clear();
  PlanOne(narrow, d, f_.budgets);
  narrow.queries += job.queries;
  narrow.log.insert(narrow.log.begin(), job.log.begin(), job.log.end());
  if (!narrow.found) narrow.note = job.note + "; without the next modules: " + narrow.note;
  if (!narrow.found) narrow.built = job.built;
  narrow.narrowed = narrow.found;
  job = narrow;
}

void Planner::PlanMoves() {
  std::vector<int> todo;
  for (size_t i = 0; i < moves_.size(); ++i) {
    if (moves_[i].in_range) todo.push_back(static_cast<int>(i));
  }
  const int threads = std::max(
      1, std::min<int>(options_.threads, static_cast<int>(todo.size())));
  std::atomic<int> next{0};
  auto worker = [&]() {
    std::vector<DataPtr> d(scenes_.size());  // One for each stage's scene.
    for (;;) {
      const int k = next.fetch_add(1);
      if (k >= static_cast<int>(todo.size())) break;
      MoveJob& job = moves_[todo[k]];
      if (!d[job.stage]) d[job.stage] = scenes_[job.stage]->MakeData();
      PlanNarrowed(job, d[job.stage].get());
    }
  };
  if (threads == 1) {
    worker();
  } else {
    std::vector<std::thread> pool;
    for (int i = 0; i < threads; ++i) pool.emplace_back(worker);
    for (std::thread& t : pool) t.join();
  }
  // Into the blocks, in order; with the torque bound also as timed unslowed.
  if (dynamics_) {
    plain_segs_.clear();
    for (const Block& b : blocks_) plain_segs_.push_back(b.segs);
  }
  for (MoveJob& job : moves_) {
    if (!job.in_range) continue;
    result_.queries += job.queries;
    MoveRow row;
    row.step = job.step;
    row.arm = scene_->arm(job.arm).use->name;
    // The torque bound: the module arm's move played slower until its
    // inverse dynamics hold the fraction of every limit.
    double time_scale = 1.0;
    if (job.found && dynamics_ && row.arm == torque_arm_) {
      Block& tb = blocks_[job.block];
      const Segment& ts = tb.segs[job.segment];
      const std::optional<Trajectory> traj = MoveTrajectory(job.path, motions_[job.arm], f_);
      TorqueDemand demand;
      std::string why = "the move cannot be timed";
      time_scale = traj ? TorqueTimeScale(*dynamics_, *traj, ts.tool, ts.held,
                                          f_.torque_limit_scale, &demand, &why)
                        : -1.0;
      if (time_scale < 0.0) {
        Fault(Fmt("step %d %s: move from %s to %s: torque bound: %s", job.step, row.arm.c_str(),
                  job.from_label.c_str(), job.to_label.c_str(), why.c_str()));
        time_scale = 1.0;
      } else if (time_scale > 1.0) {
        job.planned_s = MoveDuration(job.path, motions_[job.arm], f_) * time_scale;
        slowed_ = true;
      }
    }

    row.time_scale = time_scale;
    row.from = job.from_label;
    row.to = job.to_label;
    row.found = job.found;
    row.budget = job.budget;
    row.waypoints = static_cast<int>(job.path.size());
    row.queries = job.queries;
    row.planned_s = job.found ? job.planned_s + f_.settle_s : 0.0;
    row.model_s = job.model_s;
    row.note = job.note;
    result_.rows.push_back(row);
    if (!job.found) {
      all_found_ = false;
      Fault(Fmt("step %d %s: move from %s to %s not found: %s", job.step,
                row.arm.c_str(), job.from_label.c_str(), job.to_label.c_str(),
                job.note.c_str()));
    }
    Block& b = blocks_[job.block];
    Segment& s = b.segs[job.segment];
    s.found = job.found;
    s.budget = job.budget;
    s.queries = job.queries;
    s.planned_s = job.planned_s;
    s.time_scale = time_scale;
    s.model_s = job.model_s;
    s.seed = job.seed;
    s.built = job.built;
    if (job.found) s.path = job.path;
    // The move's duration, and every later segment of the block after it.
    const double duration = job.found ? job.planned_s + f_.settle_s : 0.0;
    const double shift = s.t0 + duration - s.t1;
    s.t1 = s.t0 + duration;
    for (size_t k = job.segment + 1; k < b.segs.size(); ++k) {
      b.segs[k].t0 += shift;
      b.segs[k].t1 += shift;
    }
    if (dynamics_) {
      // The same move unslowed, for ScheduleSlowed's order of placing.
      std::vector<Segment>& ps = plain_segs_[job.block];
      Segment& p = ps[job.segment];
      const double t0 = p.t0, t1 = p.t1;
      p = s;
      p.time_scale = 1.0;
      p.planned_s = job.planned_s / time_scale;
      p.t0 = t0;
      p.t1 = t0 + (job.found ? p.planned_s + f_.settle_s : 0.0);
      for (size_t k = job.segment + 1; k < ps.size(); ++k) {
        ps[k].t0 += p.t1 - t1;
        ps[k].t1 += p.t1 - t1;
      }
    }
  }
}

const std::vector<MovingGeom>& Planner::ArmSet(int a, const std::string& tool,
                                               const std::string& held) {
  auto key = std::make_tuple(a, tool, held);
  auto it = sets_.find(key);
  if (it != sets_.end()) return it->second;
  std::vector<MovingGeom> set = scene_->Moving(a, tool, held);
  // The tools that wait on its carriage ride with it.
  for (const auto& w : scene_->arm(a).h->waiting_tool_geoms) {
    if (w.first == tool) continue;
    for (int g : w.second) set.push_back({g, kCarriage});
  }
  return sets_[key] = set;
}

const std::vector<Obstacle>& Planner::ArmObstacles(int a,
                                                   const std::string& tool,
                                                   const std::string& held) {
  auto key = std::make_tuple(a, tool, held);
  auto it = obstacle_sets_.find(key);
  if (it != obstacle_sets_.end()) return it->second;
  std::vector<Obstacle> out;
  for (const MovingGeom& g : ArmSet(a, tool, held)) out.push_back({g.geom, 0u, 0u});
  return obstacle_sets_[key] = out;
}

void Planner::StateAt(int a, double t, Config* c, std::string* tool,
                      std::string* held, int* segment) const {
  const std::vector<Segment>& segs = rows_[a].segments;
  if (segs.empty() || t >= state_[a].free) {
    *c = state_[a].end.config;
    *tool = state_[a].end.tool;
    *held = state_[a].end.held;
    *segment = static_cast<int>(segs.size()) - 1;
    return;
  }
  // The segment with t0 <= t < t1.
  auto it = std::upper_bound(
      segs.begin(), segs.end(), t,
      [](double v, const Segment& s) { return v < s.t1; });
  if (it == segs.end()) it = segs.end() - 1;
  *segment = static_cast<int>(it - segs.begin());
  *c = ConfigAt(*it, motions_[a], f_, t);
  *tool = it->tool;
  *held = it->held;
}

Planner::Conflict Planner::Check(int a, const Block& b, double s, mjData* d) {
  Conflict out;
  const int n = scene_->ArmCount();
  const double end = s + b.segs.back().t1;
  double horizon = end;
  for (int o = 0; o < n; ++o) {
    if (o != a) horizon = std::max(horizon, state_[o].free);
  }
  const ArmMotion& m = motions_[a];
  std::vector<double> times;
  for (double t = s; t < horizon; t += f_.sample_s) times.push_back(t);
  // And the joint clock's instants k sample_s, which the verifier samples.
  for (long k = static_cast<long>(std::ceil((s + shift_) / f_.sample_s));
       static_cast<double>(k) * f_.sample_s - shift_ < horizon; ++k) {
    times.push_back(static_cast<double>(k) * f_.sample_s - shift_);
  }
  times.push_back(end);
  if (horizon > end) times.push_back(horizon);
  std::sort(times.begin(), times.end());
  size_t seg = 0;
  for (double t : times) {
    Config mine{};
    std::string tool, held, kind;
    if (t >= end) {
      const Segment& last = b.segs.back();
      mine = last.path.back();
      tool = last.tool;
      held = last.held;
      // A tool change's work ends with the other tool on the flange.
      if (report_.steps[b.step_index].kind == "tool_change" && !b.parking) {
        tool = report_.steps[b.step_index].tool_after;
      }
      kind = "end of its step";
    } else {
      const double rel = t - s;
      while (seg + 1 < b.segs.size() && rel >= b.segs[seg].t1) ++seg;
      const Segment& sg = b.segs[seg];
      mine = ConfigAt(sg, m, f_, rel);
      tool = sg.tool;
      held = sg.held;
      kind = sg.kind;
    }
    scene_->SetPose(d, a, ToPose(mine));
    struct Other {
      int arm;
      Config c;
      std::string tool, held;
      int segment;
    };
    std::vector<Other> others;
    for (int o = 0; o < n; ++o) {
      if (o == a) continue;
      Other x;
      x.arm = o;
      StateAt(o, t, &x.c, &x.tool, &x.held, &x.segment);
      scene_->SetPose(d, o, ToPose(x.c));
      others.push_back(x);
    }
    mj_kinematics(&scene_->model(), d);
    const std::vector<MovingGeom>& set = ArmSet(a, tool, held);
    // An early step (R10) against the tube on its way in.
    const double u = (t - clock_.tube_start) / layout_.tube_in.time_s;
    if (clock_.early && u >= 0.0 && u < 1.0) {
      const double apart = f_.clearance_m + f_.plan_pad_m;
      const Nearest nn = AgainstTheTubeComingIn(*scene_, d, set, u, apart);
      if (nn.distance < apart) {
        out = Conflict{true, a, t, nn, false, true, mine, mine, -1, kind};
        return out;
      }
    }
    for (const Other& x : others) {
      ++schedule_queries_;
      const double apart = f_.clearance_m + f_.plan_pad_m;
      const Nearest nn = scene_->LeastDistance(
          d, set, ArmObstacles(x.arm, x.tool, x.held), apart);
      if (nn.distance < apart) {
        out.any = true;
        out.other = x.arm;
        out.t = t;
        out.nearest = nn;
        out.standing = t >= state_[x.arm].free;
        out.mine = mine;
        out.theirs = x.c;
        out.other_segment = x.segment;
        out.my_kind = kind;
        return out;
      }
    }
  }
  return out;
}

std::string Planner::SegmentText(int a, int segment) const {
  const std::vector<Segment>& segs = rows_[a].segments;
  if (segment < 0 || segment >= static_cast<int>(segs.size())) return "its start";
  const Segment& s = segs[segment];
  auto it = index_of_step_.find(s.step);
  const std::string what =
      it != index_of_step_.end() ? report_.steps[it->second].what : "";
  return Fmt("the end of its %s of step %d (%s), from %.2f s on", s.kind.c_str(),
             s.step, what.c_str(), s.t1);
}

void Planner::Place(int a, Block& b, double s, double ready,
                    const std::string& need_reason,
                    const std::string& wait_reason) {
  ArmState& st = state_[a];
  std::vector<Segment>& row = rows_[a].segments;
  auto wait = [&](double t0, double t1, const std::string& reason) {
    if (t1 <= t0) return;
    Segment w;
    w.kind = "wait";
    w.step = b.number;
    w.t0 = t0;
    w.t1 = t1;
    w.tool = st.end.tool;
    w.held = st.end.held;
    w.path = {st.end.config};
    w.reason = reason;
    row.push_back(w);
  };
  wait(st.free, std::min(ready, s), need_reason);
  wait(std::max(st.free, ready), s, wait_reason);
  for (const Segment& sg : b.segs) {
    Segment x = sg;
    x.t0 += s;
    x.t1 += s;
    if (x.kind == "work") b.work_end = x.t1;
    row.push_back(x);
  }
  b.placed = true;
  b.start = s;
  b.end = s + b.segs.back().t1;
  st.free = b.end;
  const Segment& last = b.segs.back();
  st.end.config = last.path.back();
  st.end.tool = last.tool;
  st.end.held = last.held;
  const BayStep& step = report_.steps[b.step_index];
  if (step.kind == "tool_change" && !b.parking) st.end.tool = step.tool_after;
}

bool Planner::Schedule() {
  const int n = scene_->ArmCount();
  rows_.assign(n, {});
  state_.assign(n, {});
  // The bay's first events.
  double t = 0.0;
  BayEvent tube{"tube_in", 0, 0.0, layout_.tube_in.time_s};
  for (const BayStep& st : report_.steps) {
    if (st.kind == "tube_in") tube.step = st.number;
  }
  t = tube.t1;
  BayEvent clamp{"clamp", 0, t, t + layout_.holder.clamp_s};
  for (const BayStep& st : report_.steps) {
    if (st.kind == "clamp") clamp.step = st.number;
  }
  events_ = {tube, clamp};
  clamp_end_ = clamp.t1;
  // Where each arm starts: its wait pose, or in a part of the bay the start
  // of its first block in the part.
  for (int a = 0; a < n; ++a) {
    rows_[a].name = scene_->arm(a).use->name;
    const SceneArm& sa = scene_->arm(a);
    ArmEnd e{WaitPose(sa), sa.use->first_tool, ""};
    if (Partial()) {
      for (int bi : arm_blocks_[a]) {
        if (!blocks_[bi].in_range) continue;
        const Segment& s0 = blocks_[bi].segs.front();
        e.config = s0.kind == "move" ? s0.path.front() : s0.path.front();
        e.tool = s0.tool;
        e.held = s0.held;
        break;
      }
    }
    state_[a].end = e;
    state_[a].free = 0.0;
    Segment wseg;
    wseg.kind = "wait";
    wseg.step = clamp.step;
    wseg.t0 = 0.0;
    // A later bay's early steps (R10) may start from early_from.
    wseg.t1 = clock_.early ? std::clamp(clock_.early_from, 0.0, clamp_end_) : clamp_end_;
    wseg.tool = e.tool;
    wseg.held = e.held;
    wseg.path = {e.config};
    wseg.reason = "for the tube to come in and be clamped";
    if (wseg.t1 > 0.0) rows_[a].segments.push_back(wseg);
    state_[a].free = wseg.t1;
  }
  // The precedences of each block; the first take and the bolts by stage.
  std::map<int, int> first_take;
  std::map<int, std::vector<int>> bolts;
  std::map<int, int> rail_set_step, module_set_step, bolt_step;
  for (const BayStep& st : report_.steps) {
    const int k = stage_of_step_[st.number];
    if (st.kind == "module_take" && !first_take.count(k)) first_take[k] = st.number;
    if (st.kind == "strap_bolt") bolts[k].push_back(st.number);
    if (st.kind == "strap_bolt") bolt_step[st.index] = st.number;
    if (st.kind == "rail_set") rail_set_step[st.index] = st.number;
    if (st.kind == "module_set") module_set_step[st.index] = st.number;
  }
  // The order of work (src/layout.h, WorkOrder).
  const std::vector<int> module_order = ModuleOrder(layout_);
  const bool after_their_rails = layout_.order.modules_start == "after_their_rails";
  const bool early_pin_tools = layout_.order.pin_tools == "early";
  // Just in time: a step of phase "modules" of an arm that is neither a
  // take nor a set (a tool change, the take of a pin magazine) waits for
  // what that arm's next hole step waits for, so that the arm does not
  // stand over the lane, in the module arm's way, before its module is set.
  auto next_hole = [&](const Block& b) -> int {
    const BayStep& st = report_.steps[b.step_index];
    if (st.phase != "modules" || early_pin_tools) return -1;
    if (staged_ && st.kind == "pins_take") return -1;  // Before the modules.
    if (st.kind != "tool_change" && st.kind != "pins_take") return -1;
    for (size_t k = b.step_index + 1; k < report_.steps.size(); ++k) {
      const BayStep& q = report_.steps[k];
      if (q.arm == st.arm && IsHoleKind(q.kind)) return static_cast<int>(k);
    }
    return -1;
  };
  auto needs_of = [&](const Block& block) {
    std::vector<Need> needs;
    // A parking move waits for nothing.
    if (block.parking) return needs;
    const int hole = next_hole(block);
    Block b = block;
    if (hole >= 0) b.step_index = hole;
    const BayStep& st = report_.steps[b.step_index];
    if (st.kind == "strap_bolt") {
      auto it = rail_set_step.find(st.index);
      if (it != rail_set_step.end()) needs.push_back({it->second, true});
    } else if (IsHoleKind(st.kind)) {
      const int m = st.index / 4;
      auto it = module_set_step.find(m);
      if (it != module_set_step.end()) needs.push_back({it->second, true});
      for (int k = b.step_index - 1; k >= 0; --k) {
        const BayStep& q = report_.steps[k];
        if (IsHoleKind(q.kind) && q.index / 4 == m) {
          needs.push_back({q.number, false});
          break;
        }
      }
    } else if (st.kind == "module_take" && after_their_rails) {
      // The take of module m (rails m and m+1) after those rails' bolts.
      if (st.index >= 0 && st.index < static_cast<int>(module_order.size())) {
        const int m = module_order[st.index];
        for (int rail : {m, m + 1}) {
          auto it = bolt_step.find(rail);
          if (it != bolt_step.end()) needs.push_back({it->second, false});
        }
      }
    } else if (first_take[stage_of_step_[st.number]] == st.number) {
      for (int k : bolts[stage_of_step_[st.number]]) needs.push_back({k, false});
    }
    return needs;
  };
  DataPtr d = scene_->MakeData();
  for (;;) {
    struct Candidate {
      int arm;
      int block;
      double ready;
      std::string reason;
    };
    std::vector<Candidate> ready;
    std::string waits;  // Why each arm's next block may not start.
    bool left = false;
    for (int a = 0; a < n; ++a) {
      ArmState& st = state_[a];
      while (st.next < arm_blocks_[a].size() &&
             !blocks_[arm_blocks_[a][st.next]].in_range) {
        ++st.next;
      }
      if (st.next >= arm_blocks_[a].size()) continue;
      left = true;
      const int bi = arm_blocks_[a][st.next];
      const Block& b = blocks_[bi];
      double r = st.free;
      std::string reason;
      // R10: only a step that touches neither the tube nor the bay starts
      // before the clamp has ended.
      if (r < clamp_end_ &&
          (b.parking || TouchesTheBay(report_.steps[b.step_index].kind))) {
        r = clamp_end_;
        reason = "for the tube to come in and be clamped";
      }
      bool can = b.after < 0 || blocks_[b.after].placed;
      std::string why;
      if (!can) why = "after another block, not yet placed";
      for (const Need& need : needs_of(b)) {
        auto it = block_of_step_.find(need.step);
        if (it == block_of_step_.end()) continue;
        const Block& nb = blocks_[it->second];
        if (!nb.in_range) continue;
        if (!nb.placed) {
          can = false;
          why = Fmt("for step %d (%s), not yet placed", report_.steps[nb.step_index].number,
                    report_.steps[nb.step_index].what.c_str());
          break;
        }
        const double when = need.work_end ? nb.work_end : nb.end;
        if (when > r) {
          r = when;
          const BayStep& q = report_.steps[nb.step_index];
          reason = Fmt("for step %d (%s) by %s", q.number, q.what.c_str(),
                       q.arm.c_str());
          const int hole = next_hole(b);
          if (hole >= 0) {
            reason += Fmt(", just in time for its step %d (%s)",
                          report_.steps[hole].number,
                          report_.steps[hole].what.c_str());
          }
        }
      }
      // Staged work: after the transfer to the block's stage, and a module
      // set after the stage's magazine takes and their parks.
      const double moved = b.stage > 0 ? TransferEnd(b.stage) : 0.0;
      if (moved < 0.0) {
        can = false;
        why = "for the tube's move to stage " + scopes_[b.stage].name;
      }
      if (moved > r) {
        r = moved;
        reason = "for the tube to move to stage " + scopes_[b.stage].name;
      }
      for (const MoveJob& j : moves_) {
        const BayStep& set = report_.steps[b.step_index];
        if (!j.narrowed || set.kind != "module_set" || j.stage != b.stage ||
            std::count(j.built.modules.begin(), j.built.modules.end(), set.index) > 0) {
          continue;
        }
        const Block& mb = blocks_[j.block];
        if (can && !mb.placed) why = Fmt("for the move to step %d, not yet placed", j.step);
        can = can && mb.placed;
        if (mb.placed && mb.start + mb.segs[j.segment].t1 > r) {
          r = mb.start + mb.segs[j.segment].t1;
          reason = Fmt("for %s's move to step %d, planned without module %d",
                       scene_->arm(j.arm).use->name.c_str(), j.step, set.index);
        }
      }
      for (const Block& pb : blocks_) {
        if (!pb.magazine || pb.stage != b.stage || !pb.in_range ||
            report_.steps[b.step_index].kind != "module_set") {
          continue;
        }
        if (can && !pb.placed) {
          why = Fmt("for step %d (%s), not yet placed", report_.steps[pb.step_index].number,
                    report_.steps[pb.step_index].what.c_str());
        }
        can = can && pb.placed;
        if (pb.placed && pb.end > r) {
          r = pb.end;
          reason = "for " + scene_->arm(pb.arm).use->name + " to take its pin magazine and park";
        }
      }
      if (!release_.empty() && release_[bi] > r) {
        r = release_[bi];
        reason = "to keep its place in the schedule without the torque bound";
      }
      if (can) {
        ready.push_back({a, bi, r, reason});
      } else {
        waits += Fmt("; %s's step %d (%s) waits %s", scene_->arm(a).use->name.c_str(),
                     report_.steps[b.step_index].number,
                     report_.steps[b.step_index].what.c_str(), why.c_str());
      }
    }
    if (!left) break;
    if (ready.empty()) {
      Fault("the schedule is blocked: no arm's next step may start" + waits);
      return false;
    }
    // With a rank (ScheduleSlowed), in the order of placing it gives.
    std::stable_sort(ready.begin(), ready.end(),
                     [this](const Candidate& x, const Candidate& y) {
                       if (!rank_.empty()) return rank_[x.block] < rank_[y.block];
                       return x.ready < y.ready;
                     });
    // The ready blocks are tried in order of their earliest start (ties by
    // the layout's order of arms). A block in the way of an arm that stands
    // still for good, or one that 2,000 shifts of 0.25 s do not clear, is
    // passed over and the next is tried; only when no ready block can be
    // placed is the schedule blocked, named by the first one's conflict.
    bool placed = false;
    std::string first_block;
    int yield_to = -1, yield_arm = -1;
    for (const Candidate& c : ready) {
      Block& b = blocks_[c.block];
      double s = c.ready;
      Conflict conflict;
      bool standing = false;
      int tries = 0;
      for (; tries <= kShiftTries; ++tries) {
        conflict = Check(c.arm, b, s, d.get());
        if (!conflict.any) break;
        if (conflict.standing) {
          standing = true;
          break;
        }
        s += kShiftS;
      }
      const std::string me = scene_->arm(c.arm).use->name;
      const BayStep& st = report_.steps[b.step_index];
      if (!conflict.any) {
        std::string wait_reason;
        if (s > c.ready) {
          // Name the conflict that made it wait last.
          Conflict last = Check(c.arm, b, s - kShiftS, d.get());
          wait_reason = Fmt("for %s to pass: %s at %.2f s",
                            last.tube ? "the tube" : scene_->arm(last.other).use->name.c_str(),
                            scene_->PairText(last.nearest).c_str(), last.t);
        }
        undo_[c.block] = {rows_[c.arm].segments.size(), state_[c.arm]};
        placed_order_.push_back(c.block);
        Place(c.arm, b, s, c.ready, c.reason, wait_reason);
        state_[c.arm].next++;
        placed = true;
        break;
      }
      const std::string other = scene_->arm(conflict.other).use->name;
      const std::string text =
          standing
              ? Fmt("%s cannot start step %d (%s): %s stands at %s at %s; %s "
                    "at %.4f m at %.2f s, %s at %s (%s)",
                    me.c_str(), st.number, st.what.c_str(), other.c_str(),
                    SegmentText(conflict.other, conflict.other_segment).c_str(),
                    ConfigText(conflict.theirs).c_str(),
                    scene_->PairText(conflict.nearest).c_str(),
                    conflict.nearest.distance, conflict.t, me.c_str(),
                    ConfigText(conflict.mine).c_str(), conflict.my_kind.c_str())
              : Fmt("%s cannot start step %d (%s) within %d tries of %.2f s: "
                    "%s at %.4f m at %.2f s with %s at %s",
                    me.c_str(), st.number, st.what.c_str(), kShiftTries,
                    kShiftS, scene_->PairText(conflict.nearest).c_str(),
                    conflict.nearest.distance, conflict.t, other.c_str(),
                    ConfigText(conflict.theirs).c_str());
      if (first_block.empty()) first_block = text;
      if (standing && yield_to < 0) {
        yield_to = c.block;
        yield_arm = conflict.other;
      }
    }
    // Giving way: when nothing can be placed, the schedule is taken back to
    // before the last block of the arm that stands in the first standing
    // conflict's way, and that block is placed again only after the block
    // it was in the way of (once for each pair of blocks).
    if (!placed && yield_arm >= 0 && state_[yield_arm].next > 0) {
      const int last = arm_blocks_[yield_arm][state_[yield_arm].next - 1];
      // (In a part of the bay that block may lie outside the part, unplaced.)
      if (blocks_[last].placed && yielded_.insert({last, yield_to}).second) {
        for (int k = -1; k != last;) {
          k = placed_order_.back();
          placed_order_.pop_back();
          rows_[blocks_[k].arm].segments.resize(undo_[k].first);
          state_[blocks_[k].arm] = undo_[k].second;
          blocks_[k].placed = false;
        }
        blocks_[last].after = yield_to;
        continue;
      }
    }
    if (!placed) {
      Fault("the schedule is blocked: " + first_block);
      return false;
    }
  }
  // The end of the bay.
  double modules_end = clamp_end_;
  bool any_module_step = false;
  for (const Block& b : blocks_) {
    if (!b.placed) continue;
    if (report_.steps[b.step_index].phase == "modules") {
      modules_end = std::max(modules_end, b.end);
      any_module_step = true;
    }
  }
  double bay_end = clamp_end_;
  for (int a = 0; a < n; ++a) bay_end = std::max(bay_end, state_[a].free);
  for (const BayStep& st : report_.steps) {
    const int k = stage_of_step_[st.number];
    if (st.kind != "transfer" || TransferEnd(k) < 0.0) continue;
    BayEvent move{"transfer", st.number, TransferEnd(k) - layout_.staging->transfer_s,
                  TransferEnd(k)};
    move.from_stage = k;
    move.to_stage = k + 1;
    events_.push_back(move);
  }
  if (!Partial()) {
    BayEvent release{"release", 0, modules_end,
                     modules_end + layout_.holder.release_s};
    BayEvent out{"bay_out", 0, 0.0, 0.0};
    for (const BayStep& st : report_.steps) {
      if (st.kind == "release") release.step = st.number;
      if (st.kind == "bay_out") out.step = st.number;
    }
    out.t0 = std::max(release.t1, bay_end);
    // bay_out_when_clear (src/layout_clear_out.h): from the release's end in
    // steps of sample_s, the first start from which every arm not at its wait
    // pose stays clearance_m + plan_pad_m from the bay on its way.
    if (layout_.bay_out_when_clear && release.t1 < bay_end && f_.sample_s > 0.0) {
      DataPtr d = scene_->MakeData();
      auto at = [&](int a, double t) {
        ArmInstant x;
        int segment = -1;
        StateAt(a, t, &x.config, &x.tool, &x.held, &segment);
        const bool still = t >= state_[a].free ||
                           (segment >= 0 && rows_[a].segments[segment].kind == "wait");
        const Config w = WaitPose(scene_->arm(a));
        bool same = true;
        for (int j = 0; j < 7; ++j) same = same && std::abs(x.config[j] - w[j]) <= 1e-6;
        x.at_wait = still && same;
        return x;
      };
      const double apart = f_.clearance_m + f_.plan_pad_m;
      for (int k = 0;; ++k) {
        const double s = release.t1 + k * f_.sample_s;
        if (s >= bay_end) break;
        if (ArmsPassBay(*scene_, d.get(), s, layout_.bay_out.time_s, bay_end, f_.sample_s, apart,
                        at, true, apart)
                .clear) {
          out.t0 = s;
          break;
        }
      }
    }
    out.t1 = out.t0 + layout_.bay_out.time_s;
    events_.push_back(release);
    events_.push_back(out);
    bay_end = out.t1;
    (void)any_module_step;
  }
  for (int a = 0; a < n; ++a) {
    if (state_[a].free < bay_end) {
      Segment w;
      w.kind = "wait";
      w.step = Partial() ? 0 : events_.back().step;
      w.t0 = state_[a].free;
      w.t1 = bay_end;
      w.tool = state_[a].end.tool;
      w.held = state_[a].end.held;
      w.path = {state_[a].end.config};
      w.reason = "bay out";
      rows_[a].segments.push_back(w);
      state_[a].free = bay_end;
    }
  }
  WaitsThroughTransfers(layout_, events_, &rows_);
  return true;
}

// Takes back every placed block, for another schedule.
void Planner::ResetSchedule() {
  for (Block& b : blocks_) {
    b.placed = false;
    b.after = -1;
    b.start = b.end = b.work_end = 0.0;
  }
  placed_order_.clear();
  undo_.clear();
  yielded_.clear();
}

// The torque bound's schedule (run 047). Slowing a few of the module arm's
// moves makes it later by the seconds they add, but the earliest-start
// order of placing may then let one pin arm into a part of the bay before
// another that went first without the bound, and the other waits for it to
// pass: on v16 that turned 5.80 s of slower motion into 26.60 s of bay. So
// the bay is scheduled three times: as Schedule places it; with every move
// unslowed, which gives an order of placing; and slowed in that order (each
// block still at its earliest start that its needs and the other arms
// allow). The shorter of the first and the third is kept, the first on a
// tie or when the third is blocked. The moves and their timing are the same
// in both, so is what the arm's motors are asked; only the pin arms' blocks
// (and the module arm's waits) move.
bool Planner::ScheduleSlowed() {
  const size_t faults = result_.faults.size();
  auto bay_end = [&]() {
    double e = 0.0;
    for (const ArmState& st : state_) e = std::max(e, st.free);
    return e;
  };
  // Swaps the blocks' segments with the unslowed ones (and back).
  auto swap_segs = [&]() {
    for (size_t k = 0; k < blocks_.size(); ++k) std::swap(blocks_[k].segs, plain_segs_[k]);
  };
  rank_.clear();
  const bool first = Schedule();
  const double first_end = bay_end();
  struct Kept {
    std::vector<ArmPlan> rows;
    std::vector<ArmState> state;
    std::vector<BayEvent> events;
    std::vector<Block> blocks;
    std::vector<int> order;
    std::map<int, std::pair<size_t, ArmState>> undo;
    std::set<std::pair<int, int>> yielded;
    std::vector<std::string> faults;
  };
  const Kept kept{rows_, state_, events_, blocks_, placed_order_, undo_, yielded_, result_.faults};
  // Unslowed, for the order.
  swap_segs();
  ResetSchedule();
  const bool plain = Schedule();
  std::vector<int> order = placed_order_;
  // Each block's release: its unslowed start, plus what the slowed moves
  // that started before it in the unslowed schedule added.
  std::vector<std::pair<double, double>> added;  // (unslowed start, seconds added)
  for (size_t k = 0; k < blocks_.size(); ++k) {
    for (size_t g = 0; g < blocks_[k].segs.size(); ++g) {
      const Segment& u = blocks_[k].segs[g];
      const Segment& w = plain_segs_[k][g];
      const double more = (w.t1 - w.t0) - (u.t1 - u.t0);
      if (w.time_scale > 1.0 && more > 0.0) added.emplace_back(blocks_[k].start + u.t0, more);
    }
  }
  std::vector<double> release(blocks_.size(), 0.0);
  for (size_t k = 0; k < blocks_.size(); ++k) {
    if (!blocks_[k].placed) continue;
    release[k] = blocks_[k].start;
    for (const auto& [t, more] : added) {
      if (t < blocks_[k].start) release[k] += more;
    }
  }
  swap_segs();
  bool third = false;
  if (plain && order.size() == kept.order.size()) {
    rank_.assign(blocks_.size(), static_cast<int>(blocks_.size()));
    for (size_t k = 0; k < order.size(); ++k) rank_[order[k]] = static_cast<int>(k);
    ResetSchedule();
    result_.faults.resize(faults);
    release_ = release;
    third = Schedule();
    rank_.clear();
    release_.clear();
  }
  if (third && (!first || bay_end() < first_end)) {
    result_.faults.resize(faults);
    return true;
  }
  rows_ = kept.rows;
  state_ = kept.state;
  events_ = kept.events;
  blocks_ = kept.blocks;
  placed_order_ = kept.order;
  undo_ = kept.undo;
  yielded_ = kept.yielded;
  result_.faults = kept.faults;
  return first;
}

double Planner::TransferEnd(int s) const {
  double t = clamp_end_;
  for (int k = 0; k < s; ++k) {
    for (const Block& b : blocks_) {
      if (!b.in_range || b.stage != k) continue;
      if (!b.placed) return -1.0;
      t = std::max(t, b.end);
    }
    t += layout_.staging ? layout_.staging->transfer_s : 0.0;
  }
  return t;
}

void Planner::FillPlan() {
  Plan& plan = result_.plan;
  plan.layout = layout_.name;
  plan.layout_sha256 = sha_;
  plan.figures = f_;
  plan.arms = rows_;
  plan.events = events_;
  plan.steps.clear();
  double last_bolt = clamp_end_, modules_end = -1.0;
  for (const BayStep& st : report_.steps) {
    StepTime s;
    s.number = st.number;
    s.what = st.what;
    s.arm = st.arm;
    bool have = false;
    for (const BayEvent& e : events_) {
      if (e.step == st.number && st.arm.empty()) {
        s.start = e.t0;
        s.end = e.t1;
        have = true;
      }
    }
    auto it = block_of_step_.find(st.number);
    if (it != block_of_step_.end() && blocks_[it->second].placed) {
      s.start = blocks_[it->second].start;
      // A step whose arm parks before it starts with the parking move.
      for (const Block& pb : blocks_) {
        if (pb.parking && pb.placed && pb.number == st.number) {
          s.start = std::min(s.start, pb.start);
        }
      }
      s.end = blocks_[it->second].end;
      have = true;
      if (st.kind == "strap_bolt") last_bolt = std::max(last_bolt, s.end);
      if (st.phase == "modules") modules_end = std::max(modules_end, s.end);
    }
    if (have) plan.steps.push_back(s);
  }
  double bay_end = 0.0;
  for (const ArmPlan& a : rows_) {
    if (!a.segments.empty()) bay_end = std::max(bay_end, a.segments.back().t1);
  }
  if (modules_end < 0.0) modules_end = last_bolt;
  std::map<std::string, double> model;
  for (const auto& p : report_.phases) model[p.first] = p.second;
  const double tube = layout_.tube_in.time_s;
  plan.phases = {
      {"tube in", tube, model["tube in"]},
      {"clamp", clamp_end_ - tube, model["clamp"]},
      {"rails", last_bolt - clamp_end_, model["rails"]},
      {"modules", modules_end - last_bolt, model["modules"]},
      {"end", bay_end - modules_end, model["end"]},
  };
  if (staged_) {
    std::map<int, StepTime> times;
    for (const StepTime& st : plan.steps) times[st.number] = st;
    plan.phases = StagedPhases(layout_, report_, 0, times, events_, bay_end);
  }
  // Without a schedule there are no planned times.
  if (!scheduled_) {
    for (PhaseTime& p : plan.phases) p.planned_s = 0.0;
    bay_end = 0.0;
  }
  plan.bay_planned_s = bay_end;
  plan.bay_model_s = report_.bay_time_s;
  plan.complete = false;
}

PlanResult Planner::Run(bool plan, bool verify) {
  const auto t0 = std::chrono::steady_clock::now();
  auto done = [&]() {
    result_.seconds = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    return result_;
  };
  report_ = given_ != nullptr ? *given_ : CheckLayout(layout_, paths_);
  if (!report_.error.empty()) {
    Fault("the checker could not build the model: " + report_.error);
    return done();
  }
  if (!report_.pass) {
    int failing = 0;
    std::string first;
    for (const CheckRow& r : report_.rows) {
      if (r.pass) continue;
      if (failing++ == 0) {
        first = r.check + " " + r.item + ": " + r.detail;
      }
    }
    Fault(Fmt("the layout does not pass the checker: %d rows fail, first %s",
              failing, first.c_str()));
    return done();
  }
  std::string error;
  scenes_ = StageScenes(layout_, paths_, report_.joint_margin_deg, &error);
  scene_ = scenes_.empty() ? nullptr : scenes_.front().get();
  staged_ = layout_.staging.has_value();
  for (int k = 0; staged_ && k < StageCount(layout_); ++k) {
    scopes_.push_back(ScopeOfStage(layout_, k));
  }
  const std::vector<int> order = StagedModuleOrder(layout_);
  staged_pos_.assign(order.size(), 0);
  for (size_t k = 0; k < order.size(); ++k) staged_pos_[order[k]] = static_cast<int>(k);
  if (!scene_) {
    Fault("the scene could not be loaded: " + error);
    return done();
  }
  f_ = PlanFigures{};
  if (!options_.budgets.empty()) f_.budgets = options_.budgets;
  // Staged work: a fourth budget (src/layout_plan_stages.h), in the plan's figures.
  if (options_.budgets.empty() && layout_.staging) f_.budgets.push_back(kStagedLastBudget);
  f_.clearance_m = report_.clearance_m;
  f_.joint_margin_deg = report_.joint_margin_deg;
  if (layout_.retreat_legs) {
    f_.retreat_speed = layout_.retreat_legs->speed;
    f_.retreat_accel = layout_.retreat_legs->accel;
  }
  if (layout_.path_tries) f_.path_tries = *layout_.path_tries;
  if (layout_.torque_limit_scale) {
    f_.torque_limit_scale = *layout_.torque_limit_scale;
    torque_arm_ = ModuleArm(layout_);
    std::string why;
    dynamics_ = ArmDynamics::Create(layout_, paths_, torque_arm_, &why);
    if (!dynamics_) Fault("torque_limit_scale: " + why);
  }
  for (int a = 0; a < scene_->ArmCount(); ++a) {
    motions_.push_back(MotionOf(*scene_, a, f_));
  }
  clamp_end_ = layout_.tube_in.time_s + layout_.holder.clamp_s;
  const bool assembled = Assemble();
  prepared_ = true;
  if (!assembled) {
    FillPlan();
    return done();
  }
  if (!plan) return done();
  PlanMoves();
  if (all_found_) {
    scheduled_ = slowed_ ? ScheduleSlowed() : Schedule();
  } else {
    Fault("the schedule was not built: not every move was found");
  }
  FillPlan();
  // How long planning took, without the verifier.
  result_.seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  if (!all_found_ || !scheduled_) return result_;
  if (Partial()) {
    Fault(Fmt("a part of the bay (steps %d to %d) is never complete",
              options_.first_step, options_.last_step));
    return result_;
  }
  if (!verify) {
    result_.plan.complete = true;
    return result_;
  }
  const char* skip = std::getenv("PLAN_SKIP_VERIFY");
  if (skip != nullptr && std::string(skip) == "1") {
    Fault("not verified (PLAN_SKIP_VERIFY=1)");
    return result_;
  }
  // The verifier holds an incomplete plan as a fault: the plan is complete
  // as planned here, and is not if the verifier finds a fault.
  result_.plan.complete = true;
  result_.verify = VerifyPlan(layout_, paths_, result_.plan, sha_);
  for (const std::string& v : result_.verify.faults) {
    Fault("verify: " + v);
  }
  result_.plan.complete =
      result_.verify.ok && result_.verify.faults.empty();
  return result_;
}

std::vector<std::string> Planner::Why(const std::vector<int>& steps) {
  std::vector<std::string> out;
  Run(false);
  for (const std::string& f : result_.faults) out.push_back("fault: " + f);
  if (!prepared_) return out;
  std::vector<DataPtr> data;
  for (const auto& sc : scenes_) data.push_back(sc->MakeData());
  for (int step : steps) {
    int count = 0;
    for (MoveJob& job : moves_) {
      if (job.step != step) continue;
      ++count;
      const DataPtr& d = data[job.stage];
      const LayoutScene& sc = *scenes_[job.stage];
      const std::string arm = scene_->arm(job.arm).use->name;
      out.push_back(Fmt("step %d: %s from %s to %s, tool %s, holding %s, seed %u",
                        step, arm.c_str(), job.from_label.c_str(),
                        job.to_label.c_str(), job.tool.c_str(),
                        job.held.empty() ? "nothing" : job.held.c_str(),
                        job.seed));
      out.push_back("  start " + ConfigText(job.from));
      out.push_back("  goal  " + ConfigText(job.to));
      out.push_back(Fmt("  change (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f deg; "
                        "track %.3f m)",
                        (job.to[0] - job.from[0]) / kDeg,
                        (job.to[1] - job.from[1]) / kDeg,
                        (job.to[2] - job.from[2]) / kDeg,
                        (job.to[3] - job.from[3]) / kDeg,
                        (job.to[4] - job.from[4]) / kDeg,
                        (job.to[5] - job.from[5]) / kDeg, job.to[6] - job.from[6]));
      std::string bay = "  bay: rails";
      for (int k : job.built.rails) bay += Fmt(" %d", k);
      bay += ", modules";
      for (int k : job.built.modules) bay += Fmt(" %d", k);
      bay += Fmt(", %d on the pallets", job.built.on_pallets);
      out.push_back(bay);
      sc.AllAtWait(d.get());
      const double clear = f_.clearance_m + f_.plan_pad_m;
      const MoveWorld world(sc, job.arm, job.tool, job.held, job.built, clear);
      for (int end = 0; end < 2; ++end) {
        const Nearest n = world.Least(d.get(), end == 0 ? job.from : job.to,
                                      kSceneFar);
        out.push_back(Fmt("  at the %s: %.4f m, %s (%s)", end == 0 ? "start" : "goal",
                          n.distance, scene_->PairText(n).c_str(),
                          n.distance >= clear ? "free" : "not free"));
      }
      // The straight line, marched at the edge resolution from each end.
      const ArmMotion& m = motions_[job.arm];
      double span = 0.0;
      for (int j = 0; j < 7; ++j) {
        span += m.weights[j] * std::abs(job.to[j] - job.from[j]);
      }
      const int n = std::max(1, static_cast<int>(std::ceil(span / f_.edge_resolution)));
      auto at = [&](double u) {
        Config c{};
        for (int j = 0; j < 7; ++j) c[j] = job.from[j] + u * (job.to[j] - job.from[j]);
        return c;
      };
      bool line_free = true;
      for (int dir = 0; dir < 2; ++dir) {
        bool blocked = false;
        for (int i = 1; i < n; ++i) {
          const double u = dir == 0 ? static_cast<double>(i) / n
                                    : 1.0 - static_cast<double>(i) / n;
          const Nearest nn = world.Least(d.get(), at(u), kSceneFar);
          if (nn.distance < clear) {
            out.push_back(Fmt("  straight line marched from the %s: first blocked "
                              "at u = %.3f (0 the start, 1 the goal), %s at "
                              "%.4f m",
                              dir == 0 ? "start" : "goal", u,
                              scene_->PairText(nn).c_str(), nn.distance));
            blocked = true;
            break;
          }
        }
        if (blocked) line_free = false;
        if (!blocked && dir == 0) break;
      }
      if (line_free) {
        out.push_back(Fmt("  straight line free (%d points checked)", n - 1));
        continue;
      }
      PlanNarrowed(job, d.get());
      for (const std::string& line : job.log) out.push_back("  " + line);
      out.push_back(job.found ? Fmt("  found at budget %d, %d waypoints, %lld "
                                    "queries",
                                    job.budget, static_cast<int>(job.path.size()),
                                    static_cast<long long>(job.queries))
                              : "  not found: " + job.note);
    }
    if (count == 0) {
      out.push_back(Fmt("step %d: no move leads to it", step));
    }
  }
  return out;
}

}  // namespace

std::vector<std::string> WhyMoves(const Layout& layout, const ModelPaths& paths,
                                  const std::vector<int>& steps,
                                  const PlanOptions& options) {
  Planner planner(layout, paths, "", options);
  return planner.Why(steps);
}

PlanResult PlanLayout(const Layout& layout, const ModelPaths& paths,
                      const std::string& layout_sha256,
                      const PlanOptions& options) {
  if (options.bays <= 1 || options.first_step != 0 || options.last_step != 0) {
    Planner planner(layout, paths, layout_sha256, options);
    return planner.Run();
  }
  const auto t0 = std::chrono::steady_clock::now();
  PlanResult r = PlanBays(layout, paths, options,
                          [&](int bay, const CheckReport& report, const BayClock& clock) {
                            Planner planner(layout, paths, layout_sha256,
                                            options, bay, &report, clock);
                            return planner.Run(true, false);
                          });
  r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                  .count();
  if (!r.plan.complete) return r;
  const char* skip = std::getenv("PLAN_SKIP_VERIFY");
  if (skip != nullptr && std::string(skip) == "1") {
    r.faults.push_back("not verified (PLAN_SKIP_VERIFY=1)");
    r.plan.complete = false;
    return r;
  }
  r.verify = VerifyPlan(layout, paths, r.plan, layout_sha256);
  for (const std::string& v : r.verify.faults) r.faults.push_back("verify: " + v);
  r.plan.complete = r.verify.ok && r.verify.faults.empty();
  return r;
}

}  // namespace cell::layout
