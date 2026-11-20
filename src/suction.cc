#include "src/suction.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string_view>

namespace cell {
namespace {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// MuJoCo keeps a body's orientation as a quaternion (w, x, y, z) and a
// site's as a 3x3 matrix stored row by row.
Vector3d BodyPosition(const mjData& data, int body) {
  return Vector3d(data.xpos[3 * body], data.xpos[3 * body + 1],
                  data.xpos[3 * body + 2]);
}

Quaterniond BodyRotation(const mjData& data, int body) {
  const mjtNum* q = data.xquat + 4 * body;
  return Quaterniond(q[0], q[1], q[2], q[3]).normalized();
}

Vector3d SitePosition(const mjData& data, int site) {
  return Vector3d(data.site_xpos[3 * site], data.site_xpos[3 * site + 1],
                  data.site_xpos[3 * site + 2]);
}

Matrix3d SiteRotation(const mjData& data, int site) {
  return Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
      data.site_xmat + 9 * site);
}

std::string Format(const char* format, double a, double b) {
  char text[200];
  std::snprintf(text, sizeof(text), format, a, b);
  return text;
}

bool PositiveAndFinite(double x) { return std::isfinite(x) && x > 0.0; }

// How tool_tip stands relative to one module's glass.
struct FaceCheck {
  double gap = 0.0;     // along the module's +z from the glass; < 0 is behind
  double outside = 0.0; // how far outside the outline, in the glass's plane
  double reach = 0.0;   // straight-line distance to the nearest point of glass
  double angle = 0.0;   // between tool +Z and the module's -z
  std::optional<GripFailure::Kind> failure;  // nothing: this module qualifies
};

}  // namespace

std::optional<Suction> Suction::Create(const mjModel& model,
                                       const SuctionOptions& options,
                                       std::string* error) {
  auto fail = [&](const std::string& message) -> std::optional<Suction> {
    if (error != nullptr) *error = message;
    return std::nullopt;
  };

  if (!options.module_size.allFinite() ||
      (options.module_size.array() <= 0.0).any() ||
      !PositiveAndFinite(options.max_distance) ||
      !PositiveAndFinite(options.max_angle) ||
      !std::isfinite(options.edge_margin) || options.edge_margin < 0.0) {
    return fail("module size and tolerances must be positive and finite");
  }

  const int tool = mj_name2id(&model, mjOBJ_BODY, options.tool_body.c_str());
  if (tool < 0) return fail("no body named " + options.tool_body);

  const int tip = mj_name2id(&model, mjOBJ_SITE, options.tip_site.c_str());
  if (tip < 0) return fail("no site named " + options.tip_site);

  const int weld = mj_name2id(&model, mjOBJ_EQUALITY, options.weld.c_str());
  if (weld < 0) {
    return fail("no equality constraint named " + options.weld +
                " (include models/suction.xml in the scene)");
  }
  if (model.eq_type[weld] != mjEQ_WELD ||
      model.eq_objtype[weld] != mjOBJ_BODY ||
      model.eq_obj1id[weld] != tool) {
    return fail(options.weld + " must be a weld between bodies with body1=\"" +
                options.tool_body + "\"");
  }

  std::vector<int> modules;
  const std::string_view prefix = options.module_prefix;
  for (int body = 0; body < model.nbody; ++body) {
    const char* name = mj_id2name(&model, mjOBJ_BODY, body);
    if (name != nullptr && body != tool &&
        std::string_view(name).starts_with(prefix)) {
      modules.push_back(body);
    }
  }
  if (modules.empty()) {
    return fail("no body whose name starts with " + options.module_prefix);
  }

  return Suction(options, tool, tip, weld, std::move(modules));
}

std::optional<int> Suction::Grip(mjModel* model, mjData* data,
                                 GripFailure* failure) {
  auto fail = [&](GripFailure::Kind kind,
                  const std::string& message) -> std::optional<int> {
    if (failure != nullptr) *failure = GripFailure{kind, message};
    return std::nullopt;
  };

  if (held_) {
    return fail(GripFailure::Kind::kAlreadyHolding,
                std::string("already holding ") +
                    mj_id2name(model, mjOBJ_BODY, *held_));
  }

  // mj_step leaves the positions as they were before its last integration;
  // the grip must capture the pose the bodies have now.
  mj_kinematics(model, data);

  const Vector3d tip = SitePosition(*data, tip_);
  const Matrix3d tip_rotation = SiteRotation(*data, tip_);
  const Vector3d tool_z = tip_rotation.col(2);

  const Vector3d half = options_.module_size / 2.0;

  // Check every module; keep the best that qualifies, and the nearest of all
  // for the message if none does.
  int best = -1;
  FaceCheck best_check;
  int nearest = -1;
  FaceCheck nearest_check;
  for (int body : modules_) {
    const Quaterniond rotation = BodyRotation(*data, body);
    const Vector3d local =
        rotation.conjugate() * (tip - BodyPosition(*data, body));

    FaceCheck check;
    check.gap = local.z() - half.z();
    const double out_x = std::max(std::abs(local.x()) - half.x(), 0.0);
    const double out_y = std::max(std::abs(local.y()) - half.y(), 0.0);
    check.outside = std::hypot(out_x, out_y);
    check.reach = std::hypot(check.outside, check.gap);
    const Vector3d into_glass = -(rotation * Vector3d::UnitZ());
    check.angle = std::acos(std::clamp(tool_z.dot(into_glass), -1.0, 1.0));

    const double margin = options_.edge_margin;
    if (std::abs(check.gap) > options_.max_distance) {
      check.failure = GripFailure::Kind::kTooFar;
    } else if (std::abs(local.x()) > half.x() - margin ||
               std::abs(local.y()) > half.y() - margin) {
      check.failure = GripFailure::Kind::kOffFace;
    } else if (check.angle > options_.max_angle) {
      check.failure = GripFailure::Kind::kTooTilted;
    }

    if (!check.failure &&
        (best < 0 || std::abs(check.gap) < std::abs(best_check.gap))) {
      best = body;
      best_check = check;
    }
    if (nearest < 0 || check.reach < nearest_check.reach) {
      nearest = body;
      nearest_check = check;
    }
  }

  if (best < 0) {
    const std::string name = mj_id2name(model, mjOBJ_BODY, nearest);
    switch (*nearest_check.failure) {
      case GripFailure::Kind::kTooFar:
        return fail(GripFailure::Kind::kTooFar,
                    "tool_tip is " +
                        Format("%.4f m from the plane of the glass of the "
                               "nearest module, more than %.4f m: ",
                               std::abs(nearest_check.gap),
                               options_.max_distance) +
                        name);
      case GripFailure::Kind::kOffFace:
        return fail(GripFailure::Kind::kOffFace,
                    "tool_tip is " +
                        Format("%.4f m outside the edge of the glass "
                               "(margin %.4f m) of ",
                               nearest_check.outside, options_.edge_margin) +
                        name);
      case GripFailure::Kind::kTooTilted:
        return fail(GripFailure::Kind::kTooTilted,
                    "the tool is " +
                        Format("%.4f rad off square to the glass, more than "
                               "%.4f rad: ",
                               nearest_check.angle, options_.max_angle) +
                        name);
      case GripFailure::Kind::kAlreadyHolding:
        break;  // not produced by the face checks
    }
  }

  // Bind the weld to the module at the pose the two have now. For a weld
  // between bodies, MuJoCo 4.0.0 reads eq_data as:
  //   [0, 3)   the anchor point in body2's frame (the module)
  //   [3, 6)   the same point in body1's frame (the tool)
  //   [6, 10)  body2's orientation relative to body1, quaternion w x y z
  //   [10]     torquescale, from the model file, left as it is
  //   [11]     nonzero: mj_setConst recomputes [0, 10) from qpos0
  // and pulls the two anchor points together and body1 * relpose onto
  // body2. The anchor is put at tool_tip, where the cups hold.
  const Vector3d tool_position = BodyPosition(*data, tool_);
  const Quaterniond tool_rotation = BodyRotation(*data, tool_);
  const Vector3d module_position = BodyPosition(*data, best);
  const Quaterniond module_rotation = BodyRotation(*data, best);

  const Vector3d anchor_in_module =
      module_rotation.conjugate() * (tip - module_position);
  const Vector3d anchor_in_tool =
      tool_rotation.conjugate() * (tip - tool_position);
  const Quaterniond relative =
      (tool_rotation.conjugate() * module_rotation).normalized();

  mjtNum* weld = model->eq_data + mjNEQDATA * weld_;
  for (int i = 0; i < 3; ++i) {
    weld[i] = anchor_in_module[i];
    weld[3 + i] = anchor_in_tool[i];
  }
  weld[6] = relative.w();
  weld[7] = relative.x();
  weld[8] = relative.y();
  weld[9] = relative.z();
  weld[11] = 0;  // keep this pose even if someone calls mj_setConst
  model->eq_obj2id[weld_] = best;
  data->eq_active[weld_] = 1;

  held_ = best;
  const Quaterniond tip_quaternion(tip_rotation);
  held_at_grip_.position = tip_rotation.transpose() * (module_position - tip);
  held_at_grip_.rotation =
      (tip_quaternion.conjugate() * module_rotation).normalized();
  return best;
}

void Suction::Release(mjData* data) {
  if (!held_) return;
  data->eq_active[weld_] = 0;
  held_.reset();
}

std::optional<Pose> Suction::HeldPoseInTip(const mjData& data) const {
  if (!held_) return std::nullopt;
  const Vector3d tip = SitePosition(data, tip_);
  const Matrix3d tip_rotation = SiteRotation(data, tip_);
  Pose pose;
  pose.position = tip_rotation.transpose() * (BodyPosition(data, *held_) - tip);
  pose.rotation = (Quaterniond(tip_rotation).conjugate() *
                   BodyRotation(data, *held_))
                      .normalized();
  return pose;
}

std::optional<Pose> Suction::HeldPoseInTipAtGrip() const {
  if (!held_) return std::nullopt;
  return held_at_grip_;
}

}  // namespace cell
