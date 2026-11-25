// The gate's two figures, in one place each and held together.
//
// src/bay_hardware.h repeats the offset and the yaw that the gate accepts
// (kGateOffset, kGateYaw), because that header stands alone and the capture
// range of the locating move is worked out from them. The figures themselves
// are the owner's and live in cell::PlacementTolerance (src/trial_runner.h).
// This test fails when the two differ, so that a change of the gate cannot
// leave the capture range checked against the old figures.

#include <gtest/gtest.h>

#include "src/bay_hardware.h"
#include "src/trial_runner.h"

namespace {

TEST(LocatingGate, TheHeaderRepeatsTheGatesFigures) {
  const cell::PlacementTolerance gate;
  EXPECT_DOUBLE_EQ(cell::bay::kGateOffset, gate.position);
  EXPECT_DOUBLE_EQ(cell::bay::kGateYaw, gate.yaw);
}

}  // namespace
