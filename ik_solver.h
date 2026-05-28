/*
 * ik_solver.h — Geometric Inverse Kinematics for 5-DOF arm
 *
 * Real link lengths (confirmed from STL geometry):
 *   base (waist)    : 56 mm   — vertical offset from floor to shoulder pivot
 *   L1 (Arm_01)     : 166.7 mm  — upper arm (pitch)
 *   L2 (Arm_02)     : 115.0 mm  — forearm (pitch)
 *   L3 (Arm_03)     :  46.0 mm  — ROLL joint along L2 axis; merged into L2_EFF
 *   L4 (GripperBase):  77.0 mm  — wrist offset (pitch)
 *   L5 (Gripper)    :  67.3 mm  — tip to gripper_base pivot
 *
 * Kinematic model (2-link planar IK):
 *   • L3 is a ROLL joint — it rotates around the forearm axis, NOT a pitch joint.
 *     It does NOT change the end-effector position, only orientation.
 *     Therefore it is merged into the effective forearm length:
 *       IK_L2_EFF = IK_L2 + IK_L3  (161.0 mm)
 *   • The wrist offset (from elbow-chain tip to EE) is now only L4 + L5:
 *       IK_WRIST = IK_L4 + IK_L5   (144.3 mm)
 *   • Joint 4 (theta[4]) carries the L3 roll angle — passed through from
 *     wrist_roll, never included in position calculations.
 *
 * Conventions:
 *   • World frame: X forward, Y left, Z up.
 *   • Joint 0 (base/waist)  : rotates around Z. 0° = forward.
 *   • Joint 1 (shoulder)    : pitch. 90° = arm pointing straight up.
 *   • Joint 2 (elbow)       : pitch. 90° = forearm level with upper arm.
 *   • Joint 3 (wrist pitch) : pitch. 90° = neutral.
 *   • Joint 4 (arm3 roll)   : roll around forearm axis. 90° = neutral.
 *   • Joint 5 (gripper)     : 0 = closed, 180 = open.
 *
 * Approach:
 *   1. Compute base angle from (x,y) horizontal position.
 *   2. Compute planar reach r = sqrt(x²+y²).
 *   3. Subtract wrist offset (L4+L5) from target to get elbow-chain endpoint.
 *   4. 2-link planar IK (law of cosines) for shoulder + elbow, using L2_EFF.
 *   5. Clamp shoulder + elbow to servo limits.
 *   6. Recompute wrist pitch from CLAMPED shoulder+elbow angles so the
 *      end-effector pitch is always correct even near joint limits.
 *   7. theta[4] = wrist_roll, passed through unchanged (orientation only).
 *
 * FIX (vs old version):
 *   Previously theta3 (wrist pitch) was computed from the raw solved
 *   shoulder/elbow angles. After clamping, those raw angles no longer
 *   reflect where the arm actually is, so the wrist was wrong near limits.
 *   Now theta3 is always derived from the CLAMPED values.
 *
 * All angles returned in degrees (0–180), ready for setTarget().
 */

#pragma once
#include <math.h>
#include <Arduino.h>

// ── Physical link lengths (mm) — do not change these ──────────
constexpr float IK_BASE_H  =  56.0f;   // waist/base height
constexpr float IK_L1      = 166.7f;   // upper arm          (pitch)
constexpr float IK_L2      = 115.0f;   // forearm            (pitch)
constexpr float IK_L3      =  46.0f;   // Arm_03             (ROLL — not a pitch joint)
constexpr float IK_L4      =  77.0f;   // gripper base       (pitch offset)
constexpr float IK_L5      =  67.3f;   // gripper tip        (pitch offset)

// ── Derived IK constants ───────────────────────────────────────
// L3 is a roll joint: it adds to the physical forearm length but
// does NOT contribute to the wrist offset (no position change from roll).
constexpr float IK_L2_EFF  = IK_L2 + IK_L3;           // 161.0 mm — effective 2-link forearm
constexpr float IK_WRIST   = IK_L4 + IK_L5;            // 144.3 mm — wrist+tip offset along arm axis

// ── Result struct ─────────────────────────────────────────────
struct IKResult {
  float   theta[6];     // joint angles in degrees (indices 0-5)
  bool    reachable;
  bool    clamped;      // true if any pitch joint was clamped (wrist compensation applied)
  char    msg[64];      // diagnostic string
};

// ── Solver ────────────────────────────────────────────────────
class IKSolver {
public:
  /*
   * solve(x, y, z, pitch_deg, gripper_deg, wrist_roll_deg)
   *
   * x, y, z        : target end-effector position in mm, world frame
   *                  Z is measured from the floor (base bottom).
   * pitch_deg      : desired end-effector pitch angle (0 = horizontal,
   *                  positive = tilting down). Default 0.
   * gripper_deg    : gripper opening angle 0–180. Passed through unchanged.
   * wrist_roll_deg : Arm_03 roll angle (orientation only, 0–180).
   *                  Does NOT affect position — stored in theta[4] unchanged.
   */
  static IKResult solve(float x, float y, float z,
                        float pitch_deg      = 0.0f,
                        float gripper_deg    = 90.0f,
                        float wrist_roll_deg = 90.0f)
  {
    IKResult r;
    r.reachable = false;
    r.clamped   = false;
    for (int i = 0; i < 6; i++) r.theta[i] = 90.0f;

    // ── 1. Base rotation ─────────────────────────────────────
    // atan2(y,x) → world angle in degrees, -180…+180.
    // Servo mapping: servo 90° = arm forward (+X), 0° = left, 180° = right.
    // Reachable world angles: -90° to +90° (front hemisphere, x >= 0).
    float baseDeg = atan2f(y, x) * RAD_TO_DEG;   // -180 to +180
    float theta0;
    if (x >= 0.0f) {
      theta0 = baseDeg + 90.0f;   // normal: 0°..180°
    } else {
      // Behind the base — snap to nearest reachable edge
      theta0 = (baseDeg > 0.0f) ? 180.0f : 0.0f;
    }
    if (theta0 < 0.0f || theta0 > 180.0f) {
      snprintf(r.msg, sizeof(r.msg), "Base out of range: %.1f deg", theta0);
      return r;
    }

    // ── 2. Offset z by base height ───────────────────────────
    float zRel = z - IK_BASE_H;

    // ── 3. Wrist endpoint (subtract wrist offset from target) ─
    // IK_WRIST = L4+L5 only — L3 is roll, already in IK_L2_EFF.
    // Wrist offset is projected along the desired pitch direction.
    float pitchRad = pitch_deg * DEG_TO_RAD;
    float r_horiz  = sqrtf(x*x + y*y);   // horizontal reach to target

    // Where the 2-link chain (shoulder+elbow) tip must reach:
    float wx = r_horiz - IK_WRIST * cosf(pitchRad);
    float wz = zRel    - IK_WRIST * sinf(pitchRad);

    // ── 4. 2-link planar IK (using IK_L1 and IK_L2_EFF) ─────
    float d = sqrtf(wx*wx + wz*wz);      // distance shoulder→wrist joint

    if (d > IK_L1 + IK_L2_EFF) {
      snprintf(r.msg, sizeof(r.msg),
               "Unreachable: dist=%.1fmm > %.1fmm", d, IK_L1 + IK_L2_EFF);
      return r;
    }
    if (d < fabsf(IK_L1 - IK_L2_EFF)) {
      snprintf(r.msg, sizeof(r.msg), "Too close: dist=%.1fmm", d);
      return r;
    }

    // Elbow angle (law of cosines) — uses IK_L2_EFF
    float cosElbow = (IK_L1*IK_L1 + IK_L2_EFF*IK_L2_EFF - d*d)
                     / (2.0f * IK_L1 * IK_L2_EFF);
    cosElbow = constrain(cosElbow, -1.0f, 1.0f);
    float elbowRad = acosf(cosElbow);   // 0 = straight, π = folded

    // Shoulder angle (elbow-up solution)
    float alpha = atan2f(wz, wx);
    float cosBeta = constrain(
      (IK_L1*IK_L1 + d*d - IK_L2_EFF*IK_L2_EFF) / (2.0f * IK_L1 * d),
      -1.0f, 1.0f);
    float beta        = acosf(cosBeta);
    float shoulderRad = alpha + beta;

    // Convert to servo degrees
    float theta1 = shoulderRad * RAD_TO_DEG;
    float theta2 = 180.0f - (elbowRad   * RAD_TO_DEG);

    // ── 5. Clamp shoulder + elbow to servo limits ─────────────
    // Track whether clamping occurs so we know to recompute wrist.
    float theta1_raw = theta1;
    float theta2_raw = theta2;
    theta1 = constrain(theta1, 0.0f, 90.0f);
    theta2 = constrain(theta2, 0.0f, 90.0f);
    if (theta1 != theta1_raw || theta2 != theta2_raw) r.clamped = true;

    // ── 6. Wrist pitch — computed from CLAMPED angles ─────────
    //
    // BUG FIX: Previously wrist was computed from raw shoulderRad/elbowRad.
    // After clamping, those no longer represent where the arm actually is.
    // We must back-convert the clamped servo degrees to radians first,
    // then derive the actual arm pitch, then solve for the required wrist.
    //
    // Servo→rad back-conversion (inverse of theta1/theta2 formulas above):
    //   theta1 = degrees(shoulderRad)  →  shoulderRad = radians(theta1)
    //   theta2 = 180 - degrees(elbowRad)    →  elbowRad    = radians(180 - theta2)
    float effectiveShoulderRad = theta1 * DEG_TO_RAD;
    float effectiveElbowRad    = (180.0f - theta2) * DEG_TO_RAD;

    // Arm chain pitch = direction the wrist joint is pointing
    // (angle of the forearm tip relative to horizontal)
    float armPitch = effectiveShoulderRad + (effectiveElbowRad - (float)M_PI);

    // Required wrist angle to achieve desired EE pitch
    float wristRad = pitchRad - armPitch;
    float theta3   = 90.0f + (wristRad * RAD_TO_DEG);
    theta3         = constrain(theta3, 0.0f, 150.0f);

    // ── 7. Pack result ────────────────────────────────────────
    r.theta[0] = theta0;
    r.theta[1] = theta1;
    r.theta[2] = theta2;
    r.theta[3] = theta3;
    r.theta[4] = constrain(wrist_roll_deg, 0.0f, 180.0f);  // roll — orientation only
    r.theta[5] = constrain(gripper_deg,    0.0f, 180.0f);
    r.reachable = true;
    snprintf(r.msg, sizeof(r.msg),
             "OK base=%.0f sh=%.0f el=%.0f wr=%.0f%s",
             theta0, theta1, theta2, theta3,
             r.clamped ? " [CLAMPED]" : "");
    return r;
  }

  /*
   * Forward kinematics — returns EE position given joint angles.
   * Uses IK_L2_EFF (L2+L3) for the elbow segment.
   * Roll (theta[4]) has no effect on EE position — intentionally ignored.
   */
  static void forwardKinematics(const float th[6],
                                float& ex, float& ey, float& ez)
  {
    float baseRad = (th[0] - 90.0f) * DEG_TO_RAD;
    float shRad   = th[1]          * DEG_TO_RAD;
    float elRad   = (180.0f - th[2]) * DEG_TO_RAD;
    float wrRad   = (th[3]  - 90.0f) * DEG_TO_RAD;

    // 2D side-view (plane of the arm) — uses IK_L2_EFF
    float x2d = IK_L1     * cosf(shRad)
              + IK_L2_EFF * cosf(shRad + elRad - (float)M_PI)
              + IK_WRIST  * cosf(shRad + elRad - (float)M_PI + wrRad);
    float z2d = IK_L1     * sinf(shRad)
              + IK_L2_EFF * sinf(shRad + elRad - (float)M_PI)
              + IK_WRIST  * sinf(shRad + elRad - (float)M_PI + wrRad);

    ex = x2d * cosf(baseRad);
    ey = x2d * sinf(baseRad);
    ez = IK_BASE_H + z2d;
  }
};
