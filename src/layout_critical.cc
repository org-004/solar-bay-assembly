#include "src/layout_critical.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "src/layout_bays.h"

namespace cell::layout {
namespace {

// Times in a plan file have six decimals.
constexpr double kEps = 1e-4;
// The scheduler's step of delay while another arm is in the way
// (src/layout_plan.h, "The schedule").
constexpr double kShiftS = 0.25;

std::string F2(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.2f", v);
  return b;
}

bool IsHole(const std::string& kind) {
  return kind == "locate" || kind == "pin" || kind == "swage";
}

// One block: an arm's run of segments of one step between two waits.
struct Blk {
  int arm = -1;
  int step = 0;
  size_t seg0 = 0, seg1 = 0;  // [seg0, seg1) of the arm's segments.
  double s = 0.0, e = 0.0, work_end = 0.0;
  bool parking = false;
  int prev = -1;  // The arm's block before it, -1: none.
};

// A link to be: a block (with the instant its successor may start) or an
// event of a bay.
struct Node {
  int block = -1;
  std::string event;
  int bay = 0;
  double handover = 0.0;
};

class Walker {
 public:
  Walker(const Layout& layout, const CheckReport& report, const LayoutScene& scene,
         const PlanPlayer& player)
      : layout_(layout), report_(report), scene_(scene), player_(player),
        plan_(player.plan()), n_(static_cast<int>(report.steps.size())) {}

  bool Init(std::string* error) {
    if (layout_.staging) {
      *error = "staged work is not read";
      return false;
    }
    for (size_t i = 0; i < report_.steps.size(); ++i) local_[report_.steps[i].number] = i;
    for (size_t a = 0; a < plan_.arms.size(); ++a) {
      const std::vector<Segment>& segs = plan_.arms[a].segments;
      int last = -1;
      for (size_t i = 0; i < segs.size();) {
        if (segs[i].kind == "wait") {
          ++i;
          continue;
        }
        Blk b;
        b.arm = static_cast<int>(a);
        b.step = segs[i].step;
        b.seg0 = i;
        while (i < segs.size() && segs[i].kind != "wait" && segs[i].step == b.step) {
          if (segs[i].kind == "work") b.work_end = segs[i].t1;
          ++i;
        }
        b.seg1 = i;
        b.s = segs[b.seg0].t0;
        b.e = segs[b.seg1 - 1].t1;
        b.prev = last;
        if (last >= 0 && blocks_[last].step == b.step && blocks_[last].arm == b.arm) {
          blocks_[last].parking = true;
        }
        last = static_cast<int>(blocks_.size());
        blocks_.push_back(b);
      }
    }
    // The main block of each step (the last of its arm's blocks of it).
    for (size_t k = 0; k < blocks_.size(); ++k) block_of_step_[blocks_[k].step] = static_cast<int>(k);
    for (const Blk& b : blocks_) {
      if (Local(b.step) == nullptr) {
        *error = "step " + std::to_string(b.step) + " is not a step of the checker's bay";
        return false;
      }
    }
    return true;
  }

  int Bays() const { return plan_.bays; }

  const BayEvent* Event(const std::string& kind, int bay) const {
    for (const BayEvent& e : plan_.events) {
      if (e.kind == kind && e.bay == bay) return &e;
    }
    return nullptr;
  }

  // The checker's step for a plan step number, nullptr if none.
  const BayStep* Local(int number) const {
    const int local = number - BayOfStep(number, n_) * n_;
    auto it = local_.find(local);
    return it == local_.end() ? nullptr : &report_.steps[it->second];
  }
  int LocalIndex(int number) const { return static_cast<int>(local_.at(number - BayOfStep(number, n_) * n_)); }
  int BayOf(int number) const { return BayOfStep(number, n_); }
  std::string ArmName(int a) const { return plan_.arms[a].name; }

  // The latest block of phase "modules" of arm `a` (any arm: -1) in bay b.
  int LastModuleBlock(int bay, int a) const {
    int best = -1;
    for (size_t k = 0; k < blocks_.size(); ++k) {
      const Blk& b = blocks_[k];
      if (BayOf(b.step) != bay || (a >= 0 && b.arm != a)) continue;
      if (Local(b.step)->phase != "modules") continue;
      if (best < 0 || b.e > blocks_[best].e) best = static_cast<int>(k);
    }
    return best;
  }
  const Blk& Block(int k) const { return blocks_[k]; }

  // The chain that ends at `node`, from its root; *from_s the root's end.
  std::vector<ChainLink> Walk(Node node, double* from_s) const {
    std::vector<ChainLink> links;
    for (int guard = 0; guard < 100000; ++guard) {
      Node pred;
      ChainLink link;
      bool root = false;
      if (node.block >= 0) {
        pred = BlockLink(node, &link);
      } else {
        root = EventLink(node, &link, &pred, from_s);
      }
      links.push_back(link);
      if (root || pred.handover < 0.0) break;
      node = pred;
    }
    std::reverse(links.begin(), links.end());
    return links;
  }

 private:
  struct Cand {
    double t = 0.0;
    int prio = 0;  // 0 need, 1 clamp or boundary, 2 arm.
    std::string cause, text;
    Node pred;
  };

  int StepBlock(int number) const {
    auto it = block_of_step_.find(number);
    return it == block_of_step_.end() ? -1 : it->second;
  }

  // What step `number` needs by the scheduler's rules: (step, work_end).
  std::vector<std::pair<int, bool>> Needs(const Blk& blk) const {
    std::vector<std::pair<int, bool>> needs;
    if (blk.parking) return needs;
    const int bay = BayOf(blk.step);
    const int base = bay * n_;
    int idx = LocalIndex(blk.step);
    const BayStep* st = &report_.steps[idx];
    // Just in time: a tool change or magazine take of phase modules waits
    // for what the arm's next hole step waits for.
    if (st->phase == "modules" && layout_.order.pin_tools != "early" &&
        (st->kind == "tool_change" || st->kind == "pins_take")) {
      for (size_t k = idx + 1; k < report_.steps.size(); ++k) {
        if (report_.steps[k].arm == st->arm && IsHole(report_.steps[k].kind)) {
          idx = static_cast<int>(k);
          st = &report_.steps[k];
          break;
        }
      }
    }
    auto find = [&](const std::string& kind, int index) {
      for (const BayStep& q : report_.steps) {
        if (q.kind == kind && q.index == index) return q.number;
      }
      return -1;
    };
    if (st->kind == "strap_bolt") {
      const int k = find("rail_set", st->index);
      if (k >= 0) needs.emplace_back(base + k, true);
    } else if (IsHole(st->kind)) {
      const int m = st->index / 4;
      const int k = find("module_set", m);
      if (k >= 0) needs.emplace_back(base + k, true);
      for (int j = idx - 1; j >= 0; --j) {
        const BayStep& q = report_.steps[j];
        if (IsHole(q.kind) && q.index / 4 == m) {
          needs.emplace_back(base + q.number, false);
          break;
        }
      }
    } else if (st->kind == "module_take" &&
               layout_.order.modules_start == "after_their_rails") {
      const std::vector<int> order = ModuleOrder(layout_);
      if (st->index >= 0 && st->index < static_cast<int>(order.size())) {
        for (int rail : {order[st->index], order[st->index] + 1}) {
          const int k = find("strap_bolt", rail);
          if (k >= 0) needs.emplace_back(base + k, false);
        }
      }
    } else if (st->kind == "module_take") {
      int first = -1;
      for (const BayStep& q : report_.steps) {
        if (q.kind == "module_take") {
          first = q.number;
          break;
        }
      }
      if (first == st->number) {
        for (const BayStep& q : report_.steps) {
          if (q.kind == "strap_bolt") needs.emplace_back(base + q.number, false);
        }
      }
    }
    return needs;
  }

  std::string StepText(int number) const {
    const BayStep* st = Local(number);
    return "step " + std::to_string(number) + " (" + (st ? st->what : "?") + ")";
  }

  // The arm (or "the tube") in the way of the block put `back` seconds
  // earlier, at the scheduler's instants: the first that comes within
  // clearance_m + plan_pad_m of it; "" if none does.
  std::string InTheWay(const Blk& blk, double back) const {
    const double apart = plan_.figures.clearance_m + plan_.figures.plan_pad_m;
    const double sample = plan_.figures.sample_s;
    const double s0 = blk.s - back, s1 = blk.e - back;
    std::vector<double> ts;
    for (int k = 0; s0 + k * sample <= s1 + 1e-9; ++k) ts.push_back(s0 + k * sample);
    for (double t = std::ceil(s0 / sample) * sample; t <= s1 + 1e-9; t += sample) ts.push_back(t);
    std::sort(ts.begin(), ts.end());
    const std::string me = ArmName(blk.arm);
    if (!data_) data_ = scene_.MakeData();
    bool tube = false;
    for (double t : ts) {
      WorldAt w = player_.At(t);
      w.arms[blk.arm] = player_.At(std::min(t + back, blk.e)).arms[blk.arm];
      for (const Closest& c : InstantDistances(scene_, report_, player_, w, data_.get())) {
        if (c.by_design || c.distance >= apart) continue;
        if (c.arm == me && !c.other_arm.empty()) return c.other_arm;
        if (c.other_arm == me) return c.arm;
        if (c.arm == me && w.event == "tube_in") tube = true;
      }
      if (tube) return "the tube";
    }
    return "";
  }

  Node BlockLink(const Node& node, ChainLink* link) const {
    const Blk& blk = blocks_[node.block];
    const BayStep* st = Local(blk.step);
    const int bay = BayOf(blk.step);
    link->what = blk.parking ? "park before " + st->what : st->what;
    link->arm = ArmName(blk.arm);
    link->step = blk.step;
    link->start = blk.s;
    link->end = node.handover;
    link->block_end = blk.e;
    Split(blk, link);
    std::vector<Cand> cands;
    const bool first_in_bay = blk.prev < 0 || BayOf(blocks_[blk.prev].step) != bay;
    if (blk.prev >= 0) {
      const Blk& p = blocks_[blk.prev];
      cands.push_back({p.e, 2, "arm", "its arm's " + StepText(p.step) + " ends", Node{blk.prev, "", 0, p.e}});
    }
    const BayEvent* clamp = Event("clamp", bay);
    const BayEvent* out = bay > 0 ? Event("bay_out", bay - 1) : nullptr;
    const BayEvent* in = Event("tube_in", bay);
    if (clamp != nullptr && (blk.parking || TouchesTheBay(st->kind) || (bay == 0 && first_in_bay))) {
      if (out != nullptr && out->t1 > clamp->t1) {
        cands.push_back({out->t1, 1, "clamp", "after the clamp, the bay before's bay_out ends",
                         Node{-1, "bay_out", bay - 1, out->t1}});
      } else {
        cands.push_back({clamp->t1, 1, "clamp", "the tube's clamp ends", Node{-1, "clamp", bay, clamp->t1}});
      }
    }
    if (bay > 0 && first_in_bay && in != nullptr && out != nullptr) {
      if (out->t1 >= in->t0) {
        cands.push_back({out->t1, 1, "early", "R10: the bay before's bay_out ends",
                         Node{-1, "bay_out", bay - 1, out->t1}});
      } else {
        cands.push_back({in->t0, 1, "early", "R10: the tube starts to come in",
                         Node{-1, "tube_in", bay, in->t0}});
      }
    }
    for (const auto& [number, work_end] : Needs(blk)) {
      const int k = StepBlock(number);
      if (k < 0) continue;
      const double t = work_end ? blocks_[k].work_end : blocks_[k].e;
      cands.push_back({t, 0, "need",
                       (work_end ? "the work of " : "") + StepText(number) + " by " +
                           ArmName(blocks_[k].arm) + (work_end ? " ends" : " ends"),
                       Node{k, "", 0, t}});
    }
    if (cands.empty()) {
      link->cause = "root";
      link->text = "its start";
      return Node{-1, "", 0, -1.0};
    }
    double ready = -1.0;
    for (const Cand& c : cands) ready = std::max(ready, c.t);
    const Cand* pick = nullptr;
    for (const Cand& c : cands) {
      if (c.t < ready - kEps) continue;
      if (pick == nullptr || c.prio < pick->prio) pick = &c;
    }
    link->cause = pick->cause;
    link->text = pick->text + " at " + F2(pick->t) + " s";
    link->wait_s = blk.s - pick->t;
    if (link->wait_s > kEps) {
      // The way: the steps of kShiftS back from the start at which an arm
      // is in the way, without a break; the rest of the wait is the hold.
      int steps = 0;
      for (; (steps + 1) * kShiftS <= link->wait_s + kEps; ++steps) {
        const std::string by = InTheWay(blk, (steps + 1) * kShiftS);
        if (by.empty()) break;
        if (steps == 0) link->by = by;
      }
      link->way_s = std::min(link->wait_s, steps * kShiftS);
      link->hold_s = link->wait_s - link->way_s;
      if (link->hold_s <= kEps) link->hold_s = 0.0;
      link->wait_cause = link->way_s > 0.0 ? (link->hold_s > 0.0 ? "way+hold" : "way") : "hold";
      if (link->hold_s > 0.0) {
        link->text += "; then held " + F2(link->hold_s) +
                      " s to its unslowed place (the torque bound's order)";
      }
      if (link->way_s > 0.0) {
        link->text += "; then waits " + F2(link->way_s) + " s for " + link->by + " in its way";
      }
    } else {
      link->wait_s = 0.0;
    }
    return pick->pred;
  }

  // Splits [start, end] of a block's link by its segments' kinds.
  void Split(const Blk& blk, ChainLink* link) const {
    const std::vector<Segment>& segs = plan_.arms[blk.arm].segments;
    for (size_t i = blk.seg0; i < blk.seg1; ++i) {
      const Segment& g = segs[i];
      const double lo = std::max(g.t0, link->start), hi = std::min(g.t1, link->end);
      if (hi <= lo) continue;
      const double part = hi - lo;
      if (g.kind == "work") {
        link->work_s += part;
      } else if (g.kind == "wait") {
        link->inner_wait_s += part;
      } else {
        link->motion_s += part;
        if (g.kind == "move" && g.time_scale > 1.0 && g.t1 > g.t0) {
          const double slow = g.planned_s * (1.0 - 1.0 / g.time_scale);
          link->slowed_s += slow * part / (g.t1 - g.t0);
        }
      }
    }
  }

  // Returns true when the node is the chain's root.
  bool EventLink(const Node& node, ChainLink* link, Node* pred, double* from_s) const {
    const BayEvent* e = Event(node.event, node.bay);
    link->event = true;
    link->what = node.event + " " + std::to_string(node.bay);
    link->step = e != nullptr ? e->step : 0;
    link->start = e != nullptr ? e->t0 : 0.0;
    link->end = node.handover;
    link->block_end = e != nullptr ? e->t1 : 0.0;
    link->work_s = link->end - link->start;
    if (e == nullptr) {
      link->cause = "root";
      *from_s = link->start;
      return true;
    }
    if (node.event == "release") {
      const int k = LastModuleBlock(node.bay, -1);
      link->cause = "modules";
      if (k < 0) {
        *from_s = e->t0;
        return true;
      }
      link->text = "the bay's last block of phase modules, " + StepText(blocks_[k].step) + " by " +
                   ArmName(blocks_[k].arm) + ", ends at " + F2(blocks_[k].e) + " s";
      link->wait_s = std::max(0.0, e->t0 - blocks_[k].e);
      *pred = Node{k, "", 0, blocks_[k].e};
      return false;
    }
    if (node.event == "clamp") {
      const BayEvent* in = Event("tube_in", node.bay);
      link->cause = "tube";
      link->text = "the tube is in at " + F2(in ? in->t1 : 0.0) + " s";
      if (in == nullptr) return true;
      *pred = Node{-1, "tube_in", node.bay, in->t1};
      return false;
    }
    if (node.event == "tube_in") {
      const BayEvent* out = node.bay > 0 ? Event("bay_out", node.bay - 1) : nullptr;
      if (out == nullptr) {
        link->cause = "root";
        link->text = "the first tube comes in";
        *from_s = e->t0;
        return true;
      }
      const double empty = WaysShareZone(layout_) ? out->t1 : HolderEmptyAt(layout_, *out);
      link->cause = "holder";
      link->text = "the holder is empty at " + F2(empty) + " s";
      link->wait_s = std::max(0.0, e->t0 - empty);
      if (link->wait_s > kEps) {
        link->wait_cause = "way";
        link->by = "the bay going out";
        link->text += "; then waits " + F2(link->wait_s) + " s to pass the bay going out";
      } else {
        link->wait_s = 0.0;
      }
      *pred = Node{-1, "bay_out", node.bay - 1, empty};
      return false;
    }
    if (node.event == "bay_out") {
      const BayEvent* rel = Event("release", node.bay);
      link->cause = "release";
      if (rel == nullptr) {
        *from_s = e->t0;
        return true;
      }
      link->text = "bay " + std::to_string(node.bay) + "'s release ends at " + F2(rel->t1) + " s";
      link->wait_s = std::max(0.0, e->t0 - rel->t1);
      if (link->wait_s > kEps) {
        link->wait_cause = "way";
        link->by = "an arm near the bay's way";
        link->text += "; then waits " + F2(link->wait_s) + " s for the arms to clear its way";
      } else {
        link->wait_s = 0.0;
      }
      *from_s = rel->t1;
      return true;
    }
    link->cause = "root";
    *from_s = e->t0;
    return true;
  }

  const Layout& layout_;
  const CheckReport& report_;
  const LayoutScene& scene_;
  const PlanPlayer& player_;
  const Plan& plan_;
  const int n_;
  std::map<int, size_t> local_;
  std::vector<Blk> blocks_;
  std::map<int, int> block_of_step_;
  mutable DataPtr data_;
};

std::vector<ChainArm> ArmsOf(const std::vector<ChainLink>& links) {
  std::vector<ChainArm> arms;
  for (const ChainLink& l : links) {
    auto it = std::find_if(arms.begin(), arms.end(),
                           [&](const ChainArm& a) { return a.arm == l.arm; });
    if (it == arms.end()) {
      arms.push_back({l.arm});
      it = arms.end() - 1;
    }
    it->motion_s += l.motion_s;
    it->work_s += l.work_s;
    it->slowed_s += l.slowed_s;
    it->wait_s += l.wait_s + l.inner_wait_s;
    it->links += 1;
  }
  return arms;
}

}  // namespace

std::vector<CriticalChain> CriticalChains(const Layout& layout, const CheckReport& report,
                                          const LayoutScene& scene, const PlanPlayer& player,
                                          std::string* error) {
  Walker w(layout, report, scene, player);
  if (!w.Init(error)) return {};
  std::vector<CriticalChain> out;
  for (int bay = 0; bay < w.Bays(); ++bay) {
    const BayEvent* rel = w.Event("release", bay);
    if (rel == nullptr) {
      *error = "bay " + std::to_string(bay) + " has no release";
      return {};
    }
    CriticalChain c;
    c.bay = bay;
    c.to_s = rel->t1;
    c.links = w.Walk(Node{-1, "release", bay, rel->t1}, &c.from_s);
    c.arms = ArmsOf(c.links);
    const int last = w.LastModuleBlock(bay, -1);
    for (size_t a = 0; a < player.plan().arms.size(); ++a) {
      if (last >= 0 && w.Block(last).arm == static_cast<int>(a)) continue;
      const int k = w.LastModuleBlock(bay, static_cast<int>(a));
      if (k < 0) continue;
      const Blk& b = w.Block(k);
      double from = 0.0;
      const std::vector<ChainLink> other = w.Walk(Node{k, "", 0, b.e}, &from);
      CriticalChain::Next n{player.plan().arms[a].name, b.step, w.Local(b.step)->what, b.e,
                            b.e - from};
      for (const ChainLink& l : other) {
        const bool shared = std::any_of(c.links.begin(), c.links.end(), [&](const ChainLink& m) {
          return m.step == l.step && m.arm == l.arm && std::abs(m.start - l.start) < kEps;
        });
        if (!shared) break;
        n.shared_step = l.step;
        n.shared_until = l.end;
      }
      c.next.push_back(n);
    }
    std::sort(c.next.begin(), c.next.end(),
              [](const CriticalChain::Next& x, const CriticalChain::Next& y) { return x.end > y.end; });
    out.push_back(std::move(c));
  }
  return out;
}

}  // namespace cell::layout
