// See src/layout_plan.h. STUB written with the header; the replay's owner
// replaces it.

#include "src/layout_plan.h"

namespace cell::layout {

ArmMotion MotionOf(const LayoutScene&, int, const PlanFigures&) { return {}; }

double MoveDuration(const std::vector<Config>&, const ArmMotion&,
                    const PlanFigures&) {
  return -1.0;
}

double LegDuration(const std::vector<Config>&, double, const ArmMotion&,
                   const PlanFigures&) {
  return -1.0;
}

Config ConfigAt(const Segment&, const ArmMotion&, const PlanFigures&, double) {
  return {};
}

std::string PlanToJson(const Plan&) { return "{}"; }

std::optional<Plan> PlanFromJson(const std::string&, std::string* error) {
  if (error) *error = "PlanFromJson is not written yet";
  return std::nullopt;
}

std::string Sha256OfFile(const std::string&) { return ""; }

PlanPlayer::PlanPlayer(const LayoutScene& scene, const CheckReport& report,
                       const Plan& plan)
    : scene_(scene), report_(report), plan_(plan) {}

double PlanPlayer::duration() const { return 0.0; }

WorldAt PlanPlayer::At(double) const { return {}; }

void PlanPlayer::SetPoses(const WorldAt&, mjData*) const {}

Built PlanPlayer::BuiltAt(const WorldAt&) const { return {}; }

std::vector<Closest> InstantDistances(const LayoutScene&, const CheckReport&,
                                      const PlanPlayer&, const WorldAt&,
                                      mjData*) {
  return {};
}

VerifyReport VerifyPlan(const Layout&, const ModelPaths&, const Plan&,
                        const std::string&, const VerifyOptions&) {
  VerifyReport r;
  r.faults.push_back("VerifyPlan is not written yet");
  return r;
}

}  // namespace cell::layout
