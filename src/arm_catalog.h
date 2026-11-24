// The arms a layout may use: what each maker publishes about its arm, and
// where this repository's model of it comes from.
//
// A layout (src/layout.h) names an arm by the key of an entry here. An entry
// holds two kinds of thing. The first is the maker's datasheet: rated
// payload, reach, mass, and each joint's range and greatest speed, with the
// document they were read from. The layout checker (src/layout_check.h)
// takes payload, ranges and speeds from here and never from a model file, so
// that a figure in a check's row can be followed to its source. The second
// is how to build the arm into a model: which file describes it, what its
// six joints are called there, in order from the base, and the frame its
// tools are fixed to, whose z axis points out of the flange.
//
// Both arms' descriptions are public and fetched by pinned hash in
// MODULE.bazel: the KUKA from kroshu/kuka_robot_descriptions (Apache-2.0)
// through models/arm.xml, the Universal Robots arm from MuJoCo Menagerie
// (BSD-3-Clause). docs/layout.md names the licence and the datasheet of
// each. Angles are degrees here because datasheets give degrees.

#ifndef SRC_ARM_CATALOG_H_
#define SRC_ARM_CATALOG_H_

#include <array>
#include <string>
#include <vector>

namespace cell::layout {

struct ArmModel {
  std::string key;    // What a layout writes in "model".
  std::string maker;  // The arm as its maker names it.
  // The datasheet.
  double payload_kg = 0.0;
  double reach_m = 0.0;
  double mass_kg = 0.0;
  std::array<double, 6> joint_lo_deg{};
  std::array<double, 6> joint_hi_deg{};
  std::array<double, 6> joint_speed_deg_s{};
  std::string datasheet;  // Title, publisher, URL, date read.
  std::string licence;    // Of the description the model is built from.
  // The model.
  std::string source;                      // "kuka" (models/arm.xml) or "menagerie_ur10e".
  std::string root_body;                   // The body that is attached to the layout.
  std::array<std::string, 6> joint_names;  // In the model file, base first.
  std::string flange;       // A body or site at the flange, z pointing out.
  bool flange_is_site = false;
};

inline const std::vector<ArmModel>& ArmCatalog() {
  static const std::vector<ArmModel> catalog = {
      {.key = "kr210_r2700_2",
       .maker = "KUKA KR 210 R2700-2 (KR QUANTEC)",
       .payload_kg = 210.0,
       .reach_m = 2.701,
       .mass_kg = 1077.0,
       .joint_lo_deg = {-185.0, -140.0, -120.0, -350.0, -125.0, -350.0},
       .joint_hi_deg = {185.0, -5.0, 168.0, 350.0, 125.0, 350.0},
       .joint_speed_deg_s = {120.0, 115.0, 112.0, 179.0, 172.0, 220.0},
       .datasheet = "KR 210 R2700-2 data sheet (0000-325-858 V11.1, 26.05.2022), KUKA Deutschland GmbH (copy at teswel.ru), https://teswel.ru/upload/iblock/25d/hi001104md7ig2a0ea13s3ke45u9ovd4.pdf, read 2026-11-24",
       .licence = "Apache-2.0 (kroshu/kuka_robot_descriptions)",
       .source = "kuka",
       .root_body = "arm_base",
       .joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                       "arm_joint_4", "arm_joint_5", "arm_joint_6"},
       .flange = "arm_flange",
       .flange_is_site = false},
      {.key = "ur10e",
       .maker = "Universal Robots UR10e",
       .payload_kg = 12.5,
       .reach_m = 1.300,
       .mass_kg = 33.3,
       .joint_lo_deg = {-360.0, -360.0, -360.0, -360.0, -360.0, -360.0},
       .joint_hi_deg = {360.0, 360.0, 360.0, 360.0, 360.0, 360.0},
       .joint_speed_deg_s = {120.0, 120.0, 180.0, 180.0, 180.0, 180.0},
       .datasheet = "UR10e Technical specification (updated November 2026), Universal Robots A/S, https://www.universal-robots.com/manuals/latest/en/datasheets/ur10e/, read 2026-11-24",
       .licence = "BSD-3-Clause (google-deepmind/mujoco_menagerie)",
       .source = "menagerie_ur10e",
       .root_body = "base",
       .joint_names = {"shoulder_pan_joint", "shoulder_lift_joint",
                       "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                       "wrist_3_joint"},
       .flange = "attachment_site",
       .flange_is_site = true},
  };
  return catalog;
}

// The entry with this key, or nullptr.
inline const ArmModel* FindArmModel(const std::string& key) {
  for (const ArmModel& arm : ArmCatalog()) {
    if (arm.key == key) return &arm;
  }
  return nullptr;
}

}  // namespace cell::layout

#endif  // SRC_ARM_CATALOG_H_
