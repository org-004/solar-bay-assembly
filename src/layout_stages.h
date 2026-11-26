// The checker's work on a tube worked in stages and on two racks' gap: the
// part of src/layout_check.h that runs the checks of one place of the tube
// several times and puts their rows together. The rules are in the opening
// comment of src/layout_check.h ("Stages" and "The gap"); this header holds
// the pieces the checker and its tests share.
//
// CheckLayout (src/layout_check.h) is defined in src/layout_stages.cc. A
// layout with no staging on a base other than "two_flat_racks" goes straight
// to CheckAtOnePlace, the checks of one place of the tube, which are what
// CheckLayout was before stages; its output is unchanged. Otherwise each
// stage s is checked by CheckAtOnePlace on StageLayout(layout, s) with
// CheckOptions::stage = ScopeOfStage(layout, s); on two racks each variant of
// the gap again, on MovedRackB of the layout. The runs are independent, each
// with its own model and solver, and run side by side on a few threads; the
// rows are put together in a fixed order, so the result does not depend on
// the threads.
//
// The order of a staged bay's modules (StagedModuleOrder): the stages in
// order, and within each stage its modules in the layout's order of modules
// (src/layout.h, ModuleOrder). Where the layout's order already sets the
// stages' modules stage by stage, this is the layout's order. The k-th
// module of this order is taken at module_take point k and is set k-th.
// StageLayout gives the stage's layout this order (WorkOrder::modules), so
// that the scene's bay at a work point (src/layout_scene.h, AtWorkPoint) has
// the earlier stages' modules on before this stage's; BuiltInStage then keeps
// the earlier stages' rails and modules and drops the later stages'.

#ifndef SRC_LAYOUT_STAGES_H_
#define SRC_LAYOUT_STAGES_H_

#include <optional>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_scene.h"

namespace cell::layout {

// The checks of the tube at one place: the whole bay, or one stage when
// options.stage is set (the layout is then the stage's, StageLayout).
CheckReport CheckAtOnePlace(const Layout& layout, const ModelPaths& paths,
                            const CheckOptions& options);

// The order in which a staged bay's modules are taken and set (above);
// ModuleOrder for a layout with no staging.
std::vector<int> StagedModuleOrder(const Layout& layout);

// The layout of stage s (an index into StagesOf): AtStage, with the order of
// modules StagedModuleOrder when the layout has staging.
Layout StageLayout(const Layout& layout, int s);

// What the checker checks at stage s (src/layout_check.h, StageScope).
StageScope ScopeOfStage(const Layout& layout, int s);

// The index of the stage at which a work point is done (src/layout_check.h,
// "Stages"); 0 for a layout with no staging.
int StageOfPoint(const Layout& layout, const std::string& kind, int index);

// The bay as built at a work point in staged work: in each Built of the
// scene's step, the stage's rails_before and modules_before, and of the
// rails and modules it has, those of the stage; the pallets hold at most
// the modules not taken at the earlier stages (pallet_total less
// modules_before).
void BuiltInStage(const StageScope& stage, int pallet_total, StepScene* scene);

// The bay when the stage starts and when it ends: without a stage the bare
// tube with every module on the pallets, and the whole bay with all but
// nine; with one, the earlier stages' parts, and those and the stage's.
Built BuiltAtStageStart(const std::optional<StageScope>& stage,
                        int pallet_total);
Built BuiltAtStageEnd(const std::optional<StageScope>& stage,
                      int pallet_total);

// The hold rows of a stage (src/layout_check.h, "Stages"), for the stage's
// layout: "grip bay_hold_<k> in the holder" for each grip and "angle pin
// bay_hold_<k> at <holder|item>".
std::vector<CheckRow> StageHoldRows(const Layout& stage_layout,
                                    const Stage& stage, double clearance_m);

// One of the places of rack "b" the gap rows check.
struct GapVariant {
  std::string item;  // "gap 0 mm", "misaligned +25 mm".
  double dx = 0.0;   // MovedRackB's arguments.
  double dy = 0.0;
};

// The four variants of a layout on two racks, in the order of the rows:
// the gap at gap_lo_m and at gap_hi_m, then rack "b" misaligned by
// +misalign_m and -misalign_m at the nominal gap.
std::vector<GapVariant> GapVariants(const Layout& layout);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_STAGES_H_
