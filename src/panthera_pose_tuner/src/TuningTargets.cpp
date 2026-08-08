#include "panthera_pose_tuner/TuningTargets.hpp"

namespace panthera_pose_tuner
{
namespace
{

TuningTarget poseTarget(
  const std::string & name,
  const std::string & description,
  double x_mm,
  double y_mm,
  double z_mm,
  RpyDeg rpy_deg,
  double velocity_scale = 0.05,
  bool allow_execute = true)
{
  TuningTarget target;
  target.kind = TargetKind::Pose;
  target.name = name;
  target.description = description;
  target.xyz_mm = {x_mm, y_mm, z_mm};
  target.rpy_deg = rpy_deg;
  target.velocity_scale = velocity_scale;
  target.acceleration_scale = velocity_scale;
  target.allow_execute = allow_execute;
  return target;
}

TuningTarget jointTarget(
  const std::string & name,
  const std::string & description,
  std::array<double, 6> joints_rad,
  double velocity_scale = 0.08,
  bool allow_execute = true)
{
  TuningTarget target;
  target.kind = TargetKind::Joint;
  target.name = name;
  target.description = description;
  target.joints_rad = joints_rad;
  target.velocity_scale = velocity_scale;
  target.acceleration_scale = velocity_scale;
  target.allow_execute = allow_execute;
  return target;
}

}  // namespace

std::vector<TuningTarget> buildTuningTargets()
{
// Units:
//   xyz_mm: gripper_center target in base_link, millimeters.
//   rpy_deg: gripper_center orientation in base_link, degrees.
//
// Verified workcell coordinates:
//   OUTLET_1 cup center: (-25, -428.53, 100) mm
//   OUTLET_2 cup center: (155, -428.53, 100) mm
//   SPECTROMETER reference: (584.14, -121.03, 290) mm at laser = 150 mm
//   CLEAN reference: (250, 400, 120) mm
//
// Verified orientations:
//   OUTLET poses: yaw = -90 deg
//   SPECTROMETER poses: yaw = 0 deg
//   CLEAN poses: yaw = 90 deg
//
// Formal workflow principle:
//   safe_joint_center is only for init, reset, error recovery, or manual tuning.
//   Normal automatic cycle should not return to safe_joint_center after each action.
//
// Efficient automatic cycle:
//   outlet_wait_mid
//   -> pickFromOutlet()
//   -> placeToSpectrometer()
//   -> spectrometer_wait_ref
//   -> pickFromSpectrometer()
//   -> cleanCup()
//   -> returnCupToOutlet()
//   -> outlet_wait_mid or next outlet approach

  constexpr double OUTLET_1_X = -25.0;
  constexpr double OUTLET_2_X = 155.0;
  constexpr double OUTLET_MID_X = 65.0;
  constexpr double OUTLET_Y = -428.53;

// More IK-friendly side approach line.
  constexpr double OUTLET_APPROACH_Y = -350.0;

// Extra near point before the real cup center.
  constexpr double OUTLET_NEAR_Y = -390.0;

  constexpr double OUTLET_GRIP_Z = 140.0;
  constexpr double OUTLET_TEST_Z_160 = 160.0;
  constexpr double OUTLET_TEST_Z_150 = 150.0;
  constexpr double OUTLET_HIGH_Z = 180.0;

  constexpr double SPEC_X = 584.14;
  constexpr double SPEC_APPROACH_X = 444.14;
  constexpr double SPEC_Y_REF = -121.03;
  constexpr double SPEC_PLACE_Z = 290.0;
  constexpr double SPEC_HIGH_Z = 360.0;

  constexpr double CLEAN_X = 250.0;
  constexpr double CLEAN_Y = 400.0;
  constexpr double CLEAN_APPROACH_Y = 260.0;
  constexpr double CLEAN_DUMP_Z = 120.0;
  constexpr double CLEAN_PRE_Z = 160.0;
  constexpr double CLEAN_HIGH_Z = 240.0;

  const RpyDeg OUTLET_RPY{0.0, 0.0, -90.0};
  const RpyDeg SPEC_RPY{0.0, 0.0, 0.0};
  const RpyDeg CLEAN_RPY{0.0, 0.0, 90.0};

  return {
// ============================================================
// 0. Safe pose and efficient wait poses
// ============================================================

    jointTarget(
      "safe_joint_center",
      "Safe joint pose for init, reset, error recovery, and manual tuning only.",
      {0.0, 0.18, 0.18, 0.0, 0.0, 0.0},
      0.08),

    poseTarget(
      "outlet_wait_mid",
      "Efficient idle/wait pose near both outlets. Use this instead of safe_joint_center during normal automatic cycle.",
      OUTLET_MID_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "spectrometer_wait_ref",
      "Efficient wait pose near spectrometer during detection at laser reference 150 mm. Hold here while waiting for detection done.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

// ============================================================
// 1. Original verified check poses
// Keep these for manual comparison and calibration.
// Do not use these directly in formal automatic workflow.
// ============================================================

    poseTarget(
      "outlet_1_high_check",
      "OUTLET_1 verified high check pose.",
      OUTLET_1_X, OUTLET_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_grip_check",
      "OUTLET_1 verified grip candidate.",
      OUTLET_1_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_high_check",
      "OUTLET_2 verified high check pose.",
      OUTLET_2_X, OUTLET_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_grip_check",
      "OUTLET_2 verified grip candidate.",
      OUTLET_2_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "spectrometer_high_check",
      "Spectrometer verified high check pose. Y must later be corrected from laser distance.",
      SPEC_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_place_check",
      "Spectrometer verified placement candidate at laser reference 150 mm.",
      SPEC_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "clean_high_check",
      "Cleaning area verified high check pose.",
      CLEAN_X, CLEAN_Y, CLEAN_HIGH_Z,
      CLEAN_RPY),

    poseTarget(
      "clean_dump_check",
      "Cleaning area verified dump candidate.",
      CLEAN_X, CLEAN_Y, CLEAN_DUMP_Z,
      CLEAN_RPY),

// ============================================================
// 2. OUTLET_1 pick path
//
// Formal sequence:
// outlet_wait_mid or previous state
// -> outlet_1_approach_high      moveJ
// -> outlet_1_pre_grasp          moveL
// -> outlet_1_near_grasp         moveL
// -> outlet_1_grasp              moveL
// -> close_gripper
// -> attach_cup
// -> outlet_1_lift               moveL
//
// Do not return to safe_joint_center here.
// Next normal state is spectrometer_place_approach_high_ref.
// ============================================================

    poseTarget(
      "outlet_1_approach_high",
      "OUTLET_1 side approach high. More IK-friendly Y=-350.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_grasp_z160",
      "OUTLET_1 pre-grasp test at z160. Manual tuning only.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_160,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_grasp_z150",
      "OUTLET_1 pre-grasp test at z150. Manual tuning only.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_150,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_grasp",
      "OUTLET_1 formal pre-grasp at final grip height, before horizontal approach.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_near_grasp",
      "OUTLET_1 near-grasp before entering the real cup center.",
      OUTLET_1_X, OUTLET_NEAR_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_grasp",
      "OUTLET_1 real grasp pose.",
      OUTLET_1_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_lift",
      "OUTLET_1 vertical lift after grasp. Next state should go directly to spectrometer place approach.",
      OUTLET_1_X, OUTLET_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_retreat_high",
      "OUTLET_1 high retreat after lifting. Usually only used for manual tuning.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

// ============================================================
// 3. OUTLET_2 pick path
// Same logic as OUTLET_1.
// ============================================================

    poseTarget(
      "outlet_2_approach_high",
      "OUTLET_2 side approach high. More IK-friendly Y=-350.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_grasp_z160",
      "OUTLET_2 pre-grasp test at z160. Manual tuning only.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_160,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_grasp_z150",
      "OUTLET_2 pre-grasp test at z150. Manual tuning only.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_150,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_grasp",
      "OUTLET_2 formal pre-grasp at final grip height, before horizontal approach.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_near_grasp",
      "OUTLET_2 near-grasp before entering the real cup center.",
      OUTLET_2_X, OUTLET_NEAR_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_grasp",
      "OUTLET_2 real grasp pose.",
      OUTLET_2_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_lift",
      "OUTLET_2 vertical lift after grasp. Next state should go directly to spectrometer place approach.",
      OUTLET_2_X, OUTLET_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_retreat_high",
      "OUTLET_2 high retreat after lifting. Usually only used for manual tuning.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

// ============================================================
// 4. Spectrometer place path at laser reference 150 mm
//
// Formal workflow must replace SPEC_Y_REF with:
//   y_spec_place = -121.03 + sign * (laser_place_mm - 150.0)
//
// Formal sequence after outlet_i_lift:
// -> spectrometer_place_approach_high_ref   moveJ
// -> spectrometer_place_pre_ref             moveL
// -> spectrometer_place_ref                 moveL
// -> open_gripper
// -> detach_cup
// -> spectrometer_place_retreat_ref         moveL
// -> spectrometer_wait_ref                  moveL or hold if same pose
//
// Then hold at spectrometer_wait_ref during detection.
// ============================================================

    poseTarget(
      "spectrometer_place_approach_high_ref",
      "Spectrometer place approach high at laser reference 150 mm.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_place_pre_ref",
      "Spectrometer pre-place at laser reference 150 mm.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_place_ref",
      "Spectrometer real place pose at laser reference 150 mm.",
      SPEC_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_place_retreat_ref",
      "Spectrometer retreat after placing.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_place_retreat_high_ref",
      "Spectrometer high retreat after placing. Same coordinates as spectrometer_wait_ref.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

// ============================================================
// 5. Spectrometer pick path at laser reference 150 mm
//
// Formal workflow must re-read laser after detection:
//   y_spec_pick = -121.03 + sign * (laser_pick_mm - 150.0)
//
// Do not reuse y_spec_place for picking.
//
// Formal sequence after detection:
// spectrometer_wait_ref
// -> spectrometer_pick_approach_high_ref    moveJ or hold if same pose
// -> spectrometer_pick_pre_ref              moveL
// -> spectrometer_pick_ref                  moveL
// -> close_gripper
// -> attach_cup
// -> spectrometer_pick_lift_ref             moveL
//
// Then go directly to clean_approach_high.
// ============================================================

    poseTarget(
      "spectrometer_pick_approach_high_ref",
      "Spectrometer pick approach high at laser reference 150 mm. Same coordinates as spectrometer_wait_ref.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_pick_pre_ref",
      "Spectrometer pre-pick at laser reference 150 mm.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_pick_ref",
      "Spectrometer real pick pose at laser reference 150 mm.",
      SPEC_X, SPEC_Y_REF, SPEC_PLACE_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_pick_lift_ref",
      "Spectrometer vertical lift after picking. Next state should go directly to clean_approach_high.",
      SPEC_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

    poseTarget(
      "spectrometer_pick_retreat_high_ref",
      "Spectrometer high retreat after picking. Usually only used for manual tuning.",
      SPEC_APPROACH_X, SPEC_Y_REF, SPEC_HIGH_Z,
      SPEC_RPY),

// ============================================================
// 6. Cleaning path
//
// Formal sequence after spectrometer_pick_lift_ref:
// -> clean_approach_high      moveJ
// -> clean_pre                moveL
// -> clean_ready_high         moveL
// -> clean_dump               moveL
// -> wrist_pour
// -> clean_ready_high         moveL
// -> clean_pre                moveL
// -> clean_retreat_high       moveL
//
// Then go directly to outlet_i_return_approach_high.
// ============================================================

    poseTarget(
      "clean_approach_high",
      "Cleaning side approach high.",
      CLEAN_X, CLEAN_APPROACH_Y, CLEAN_HIGH_Z,
      CLEAN_RPY),

    poseTarget(
      "clean_pre",
      "Cleaning side pre pose before entering dump area.",
      CLEAN_X, CLEAN_APPROACH_Y, CLEAN_PRE_Z,
      CLEAN_RPY),

    poseTarget(
      "clean_ready_high",
      "Cleaning ready high above dump position.",
      CLEAN_X, CLEAN_Y, CLEAN_PRE_Z,
      CLEAN_RPY),

    poseTarget(
      "clean_dump",
      "Cleaning dump pose. Do wrist pour here.",
      CLEAN_X, CLEAN_Y, CLEAN_DUMP_Z,
      CLEAN_RPY),

    poseTarget(
      "clean_retreat_high",
      "Cleaning high retreat after dump. Next state should go directly to outlet return approach.",
      CLEAN_X, CLEAN_APPROACH_Y, CLEAN_HIGH_Z,
      CLEAN_RPY),

// ============================================================
// 7. OUTLET_1 return path
//
// Formal sequence after clean_retreat_high:
// -> outlet_1_return_approach_high    moveJ
// -> outlet_1_pre_return              moveL
// -> outlet_1_near_return             moveL
// -> outlet_1_return                  moveL
// -> open_gripper
// -> detach_cup
// -> outlet_1_return_retreat          moveL
// -> outlet_1_return_retreat_high     moveL
//
// Then:
//   if next task exists: go directly to next outlet approach
//   else: go to outlet_wait_mid
// ============================================================

    poseTarget(
      "outlet_1_return_approach_high",
      "OUTLET_1 return approach high.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_return_z160",
      "OUTLET_1 pre-return test at z160. Manual tuning only.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_160,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_return_z150",
      "OUTLET_1 pre-return test at z150. Manual tuning only.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_150,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_pre_return",
      "OUTLET_1 formal pre-return before horizontal place.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_near_return",
      "OUTLET_1 near-return before final place.",
      OUTLET_1_X, OUTLET_NEAR_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_return",
      "OUTLET_1 real return pose.",
      OUTLET_1_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_return_retreat",
      "OUTLET_1 return horizontal retreat.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_1_return_retreat_high",
      "OUTLET_1 return retreat high.",
      OUTLET_1_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

// ============================================================
// 8. OUTLET_2 return path
// Same logic as OUTLET_1 return path.
// ============================================================

    poseTarget(
      "outlet_2_return_approach_high",
      "OUTLET_2 return approach high.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_return_z160",
      "OUTLET_2 pre-return test at z160. Manual tuning only.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_160,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_return_z150",
      "OUTLET_2 pre-return test at z150. Manual tuning only.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_TEST_Z_150,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_pre_return",
      "OUTLET_2 formal pre-return before horizontal place.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_near_return",
      "OUTLET_2 near-return before final place.",
      OUTLET_2_X, OUTLET_NEAR_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_return",
      "OUTLET_2 real return pose.",
      OUTLET_2_X, OUTLET_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_return_retreat",
      "OUTLET_2 return horizontal retreat.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_GRIP_Z,
      OUTLET_RPY),

    poseTarget(
      "outlet_2_return_retreat_high",
      "OUTLET_2 return retreat high.",
      OUTLET_2_X, OUTLET_APPROACH_Y, OUTLET_HIGH_Z,
      OUTLET_RPY)

  };
}

}  // namespace panthera_pose_tuner
