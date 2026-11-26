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
// The arms' descriptions are public and fetched by pinned hash in
// MODULE.bazel: the three KUKA arms (KR 210 R2700-2, KR 10 R1100-2 and
// KR 20 R1810-2) from kroshu/kuka_robot_descriptions (Apache-2.0) through
// models/arm.xml, models/kr10_r1100_2.xml and models/kr20_r1810_2.xml, the
// Universal Robots arm from MuJoCo Menagerie (BSD-3-Clause). docs/layout.md
// names the licence and the datasheet of each, and why the two smaller KUKA
// arms are here ("Smaller KUKA arms"). Angles are degrees here because
// datasheets give degrees.

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
  // Where the rated payload holds: the distance of the load's centre of mass
  // from the flange along the flange's axis and sideways from it, and the
  // datasheet's words for its conditions. 0 where the datasheet states no
  // distance (or it has not been read): the payload is then held by mass
  // alone.
  double payload_centre_axial_m = 0.0;
  double payload_centre_lateral_m = 0.0;
  std::string payload_conditions;
  double reach_m = 0.0;
  double mass_kg = 0.0;
  std::array<double, 6> joint_lo_deg{};
  std::array<double, 6> joint_hi_deg{};
  std::array<double, 6> joint_speed_deg_s{};
  std::string datasheet;  // Title, publisher, URL, date read.
  std::string licence;    // Of the description the model is built from.
  // The model.
  // "kuka" (models/arm.xml), "kuka_kr10_r1100_2" (models/kr10_r1100_2.xml),
  // "kuka_kr20_r1810_2" (models/kr20_r1810_2.xml) or "menagerie_ur10e".
  std::string source;
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
       .payload_centre_axial_m = 0.310,
       .payload_centre_lateral_m = 0.355,
       .payload_conditions =
           "The payload diagram's 210 kg curve, read by eye to about 10 mm: "
           "flat at Lxy 355 mm out to Lz 310 mm, then down to Lz 430 mm at Lxy 0. "
           "\"The maximum payload of 275 kg applies only if the position of the "
           "center of mass is 0 mm and a supplementary load optimized for the load "
           "case is mounted.\" \"The specific load case must be verified using "
           "KUKA.Load or KUKA Compose.\" The diagram does not print what Lz is "
           "measured from; the flange is ours.",
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
      {.key = "kr10_r1100_2",
       .maker = "KUKA KR 10 R1100-2 (KR AGILUS-2)",
       .payload_kg = 5.0,
       .payload_centre_axial_m = 0.260,
       .payload_centre_lateral_m = 0.180,
       .payload_conditions =
           "The payload diagram's 5 kg curve, the rated payload, read by eye "
           "to about 10 mm: flat at Lxy 180 mm out to Lz 260 mm, then down to "
           "Lz 310 mm at Lxy 0. The Lxy axis is printed with ticks every "
           "25 mm up to 150 and the top label reads 200 where the ticks put "
           "175 mm; the curve is read from the ticks, so it is 180 mm, and "
           "200 mm if the label is right. \"The KR 10 R1100-2 is designed for "
           "a rated payload of 5 kg in order to optimize the dynamic "
           "performance of the robot. With reduced load center distances, "
           "higher loads up to the maximum payload may be used.\" The "
           "maximum payload is 11.1 kg; the curves for 6 to 10.5 kg have their "
           "flat parts at Lxy 160 down to 96 mm and end on the axis at Lz 245 "
           "down to 75 mm. \"The specific "
           "load case must be verified using KUKA.Load.\" The diagram does "
           "not print what Lz is measured from; the flange is ours.",
       .reach_m = 1.101,
       .mass_kg = 55.0,
       .joint_lo_deg = {-170.0, -190.0, -120.0, -185.0, -120.0, -350.0},
       .joint_hi_deg = {170.0, 45.0, 156.0, 185.0, 120.0, 350.0},
       .joint_speed_deg_s = {300.0, 225.0, 330.0, 360.0, 360.0, 433.0},
       .datasheet = "KR 10 R1100-2 data sheet (0000-290-003 V17.1, 03.12.2019), KUKA Deutschland GmbH (copy at hci.rwth-aachen.de), https://hci.rwth-aachen.de/files/200311-KUKA-KR-10-R1100-2-Industrial-Robot-Specs-0000290003_EN.pdf, read 2026-11-26; the data sheet has no speeds: those are the axis data of the KR AGILUS-2 as a reseller prints them (https://www.eurobots.net/kuka-agilus-kr10-r1100-2-krc4-krc5-en.html, read 2026-11-26), and agree with the vendor's URDF velocity limits",
       .licence = "Apache-2.0 (kroshu/kuka_robot_descriptions)",
       .source = "kuka_kr10_r1100_2",
       .root_body = "arm_base",
       .joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                       "arm_joint_4", "arm_joint_5", "arm_joint_6"},
       .flange = "arm_flange",
       .flange_is_site = false},
      {.key = "kr20_r1810_2",
       .maker = "KUKA KR 20 R1810-2 (KR CYBERTECH-2)",
       .payload_kg = 20.0,
       .payload_centre_axial_m = 0.155,
       .payload_centre_lateral_m = 0.120,
       .payload_conditions =
           "The payload diagram's 20 kg curve, the rated payload, read by eye "
           "to about 10 mm: flat at Lxy 120 mm out to Lz 155 mm, then down to "
           "Lz 175 mm at Lxy 0. \"The KR 20 R1810-2 is designed for a rated "
           "payload of 20 kg in order to optimize the dynamic performance of "
           "the robot. With reduced load center distances, higher loads up to "
           "the maximum payload may be used.\" The maximum payload is 23.9 kg; "
           "the 22 kg curve is at Lxy 110 mm and ends on the axis at Lz 85 mm. "
           "\"The specific load case "
           "must be verified using KUKA.Load.\" The diagram does not print "
           "what Lz is measured from; the flange is ours.",
       .reach_m = 1.813,
       .mass_kg = 255.0,
       .joint_lo_deg = {-185.0, -185.0, -138.0, -350.0, -130.0, -350.0},
       .joint_hi_deg = {185.0, 65.0, 175.0, 350.0, 130.0, 350.0},
       .joint_speed_deg_s = {200.0, 175.0, 190.0, 430.0, 430.0, 630.0},
       .datasheet = "KR 20 R1810-2 data sheet (0000-321-012 V13.1, 10.04.2020), KUKA Deutschland GmbH (copy at quicktimeonline.com), https://www.quicktimeonline.com/assets/images/pdf/KUKA/KR%2020%20R1810-2.pdf, read 2026-11-26",
       .licence = "Apache-2.0 (kroshu/kuka_robot_descriptions)",
       .source = "kuka_kr20_r1810_2",
       .root_body = "arm_base",
       .joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                       "arm_joint_4", "arm_joint_5", "arm_joint_6"},
       .flange = "arm_flange",
       .flange_is_site = false},
      {.key = "ur10e",
       .maker = "Universal Robots UR10e",
       .payload_kg = 12.5,
       .payload_centre_axial_m = 0.0,
       .payload_centre_lateral_m = 0.0,
       .payload_conditions =
           "The technical specification states the payload without a load "
           "centre; any mounting orientation.",
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
