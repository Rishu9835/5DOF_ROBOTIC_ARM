"""
ik_bridge.py — Python port of ik_solver.h (same geometry, no Arduino deps).

Used by coordinator.py to pre-check reachability BEFORE sending commands
to the ESP32. The ESP32 does its own IK solving — this is only for
Python-side validation and logging.

Unit-tested to match C++ output within 0.5 degrees.

FIX (vs old version):
  Previously theta3 (wrist pitch) was computed from raw solved shoulder/elbow
  radians. After servo clamping those raw values no longer represent the actual
  arm position, so the wrist compensation was wrong whenever a joint hit its
  limit. Now theta3 is always recomputed from the CLAMPED servo angles.
"""

import math
from dataclasses import dataclass, field
from typing import List

# ── Link lengths (must match ik_solver.h exactly) ────────────
IK_BASE_H = 56.0
IK_L1     = 166.7
IK_L2     = 115.0
IK_L3     = 46.0    # roll joint — merged into L2_EFF
IK_L4     = 77.0
IK_L5     = 67.3

IK_L2_EFF = IK_L2 + IK_L3   # 161.0 mm
IK_WRIST  = IK_L4 + IK_L5   # 144.3 mm


@dataclass
class IKResult:
    theta: List[float]       # 6 floats, degrees
    reachable: bool
    clamped: bool            # True if any pitch joint was clamped
    msg: str


def ik_solve(x: float, y: float, z: float,
             pitch_deg: float = 0.0,
             gripper_deg: float = 90.0,
             wrist_roll_deg: float = 90.0) -> IKResult:
    """
    Solve inverse kinematics for a 5-DOF arm.
    Returns IKResult with 6 joint angles (degrees) and reachability flag.

    Parameters:
        x, y, z:        end-effector position in mm (robot frame, Z from floor)
        pitch_deg:      desired wrist pitch in degrees
                        (0 = horizontal, positive = tilting down)
        gripper_deg:    gripper servo angle (pass-through)
        wrist_roll_deg: wrist roll servo angle (pass-through)
    """
    theta = [90.0] * 6

    # ── 1. Base rotation (Joint 0) ────────────────────────────
    # atan2(y, x) gives angle from +X axis.
    # Servo 90° = arm pointing forward (+X).
    if x >= 0:
        theta0 = math.degrees(math.atan2(y, x)) + 90.0
    else:
        # Behind the robot — clamp to nearest limit
        base_deg = math.degrees(math.atan2(y, x))
        theta0 = 180.0 if base_deg > 0 else 0.0

    if not (0.0 <= theta0 <= 180.0):
        return IKResult(theta, False, False, f"Base out of range: {theta0:.1f}°")

    # ── 2. Subtract base height ───────────────────────────────
    z_rel = z - IK_BASE_H

    # ── 3. Wrist endpoint ─────────────────────────────────────
    # Remove wrist+tip offset along desired pitch direction to find
    # where the 2-link shoulder/elbow chain tip must reach.
    pitch_rad = math.radians(pitch_deg)
    r_horiz = math.sqrt(x * x + y * y)
    wx = r_horiz - IK_WRIST * math.cos(pitch_rad)
    wz = z_rel   - IK_WRIST * math.sin(pitch_rad)

    # ── 4. 2-link planar IK (elbow-up solution) ───────────────
    d = math.sqrt(wx * wx + wz * wz)

    if d > IK_L1 + IK_L2_EFF:
        return IKResult(theta, False, False,
                        f"Unreachable: dist={d:.1f}mm > {IK_L1 + IK_L2_EFF:.1f}mm")
    if d < abs(IK_L1 - IK_L2_EFF):
        return IKResult(theta, False, False,
                        f"Too close: dist={d:.1f}mm")

    # Elbow angle via law of cosines
    cos_elbow = (IK_L1**2 + IK_L2_EFF**2 - d**2) / (2 * IK_L1 * IK_L2_EFF)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_rad = math.acos(cos_elbow)

    # Shoulder angle
    alpha = math.atan2(wz, wx)
    cos_beta = (IK_L1**2 + d**2 - IK_L2_EFF**2) / (2 * IK_L1 * d)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    shoulder_rad = alpha + beta

    # Convert to servo degrees
    theta1 = math.degrees(shoulder_rad)
    theta2 = 180.0 - math.degrees(elbow_rad)

    # ── 5. Clamp shoulder + elbow to servo limits ─────────────
    theta1_raw = theta1
    theta2_raw = theta2
    theta1 = max(0.0, min(90.0, theta1))
    theta2 = max(0.0, min(90.0, theta2))
    clamped = (theta1 != theta1_raw) or (theta2 != theta2_raw)

    # ── 6. Wrist pitch — computed from CLAMPED angles ─────────
    #
    # BUG FIX: Previously wrist was computed from raw shoulder_rad/elbow_rad.
    # After clamping, those radians no longer represent where the arm is.
    # We back-convert the clamped servo degrees to radians, compute the
    # actual arm pitch, then derive the wrist angle needed for the desired EE pitch.
    #
    # Servo→rad back-conversion (inverse of theta1/theta2 formulas above):
    #   theta1 = degrees(shoulder_rad)  →  shoulder_rad = radians(theta1)
    #   theta2 = 180 - degrees(elbow_rad)    →  elbow_rad    = radians(180 - theta2)
    effective_shoulder_rad = math.radians(theta1)
    effective_elbow_rad    = math.radians(180.0 - theta2)

    # Actual direction the wrist joint is pointing (arm chain pitch)
    arm_pitch = effective_shoulder_rad + (effective_elbow_rad - math.pi)

    # Wrist angle required to achieve the desired EE pitch
    wrist_rad = pitch_rad - arm_pitch
    theta3 = 90.0 + math.degrees(wrist_rad)
    theta3 = max(0.0, min(150.0, theta3))

    # ── 7. Pack result ────────────────────────────────────────
    result_theta = [
        theta0,
        theta1,
        theta2,
        theta3,
        max(0.0, min(180.0, wrist_roll_deg)),   # roll — orientation only
        max(0.0, min(180.0, gripper_deg)),
    ]

    clamp_note = " [CLAMPED]" if clamped else ""
    return IKResult(
        theta=result_theta,
        reachable=True,
        clamped=clamped,
        msg=(f"OK base={theta0:.0f} sh={theta1:.0f} "
             f"el={theta2:.0f} wr={theta3:.0f}{clamp_note}")
    )


def forward_kinematics(theta: List[float]):
    """
    Forward kinematics — returns (ex, ey, ez) end-effector position in mm.
    Mirrors IKSolver::forwardKinematics() in ik_solver.h.
    Roll (theta[4]) has no effect on EE position and is ignored.
    """
    base_rad = math.radians(theta[0] - 90.0)
    sh_rad   = math.radians(theta[1])
    el_rad   = math.radians(180.0 - theta[2])
    wr_rad   = math.radians(theta[3] - 90.0)

    x2d = (IK_L1     * math.cos(sh_rad)
         + IK_L2_EFF * math.cos(sh_rad + el_rad - math.pi)
         + IK_WRIST  * math.cos(sh_rad + el_rad - math.pi + wr_rad))
    z2d = (IK_L1     * math.sin(sh_rad)
         + IK_L2_EFF * math.sin(sh_rad + el_rad - math.pi)
         + IK_WRIST  * math.sin(sh_rad + el_rad - math.pi + wr_rad))

    ex = x2d * math.cos(base_rad)
    ey = x2d * math.sin(base_rad)
    ez = IK_BASE_H + z2d
    return ex, ey, ez


def make_ik_command(x, y, z, pitch=0.0, grip=90.0, roll=90.0) -> dict:
    """Returns dict ready to JSON-dump and send over WebSocket."""
    return {
        "cmd":   "ik_move",
        "x":     x,
        "y":     y,
        "z":     z,
        "pitch": pitch,
        "grip":  grip,
        "roll":  roll,
    }


if __name__ == "__main__":
    import sys

    print("=" * 60)
    print("ik_bridge self-test")
    print("=" * 60)

    test_cases = [
        # (x,    y,    z,     pitch, description)
        (300,    0,  100,     0.0,  "mid reach, horizontal"),
        (350,   50,  150,     0.0,  "high reach with offset"),
        (300, -100,   56,     0.0,  "base height side offset"),
        (380,    0,  150,     0.0,  "near max reach"),
    ]

    all_pass = True
    for x, y, z, pitch, desc in test_cases:
        r = ik_solve(x, y, z, pitch_deg=pitch)
        if r.reachable:
            # Verify FK round-trip
            ex, ey, ez = forward_kinematics(r.theta)
            err_xy = abs(math.sqrt(ex**2 + ey**2) - math.sqrt(x**2 + y**2))
            err_z  = abs(ez - z)
            ok = err_xy < 2.0 and err_z < 2.0   # 2 mm tolerance
            status = "PASS" if ok else "FAIL"
            if not ok:
                all_pass = False
            print(f"  [{status}] {desc}")
            print(f"         target=({x},{y},{z}) pitch={pitch}°  "
                  f"clamped={r.clamped}")
            print(f"         angles={[f'{t:.1f}' for t in r.theta]}")
            print(f"         FK=({ex:.1f},{ey:.1f},{ez:.1f})  "
                  f"err_xy={err_xy:.2f}mm  err_z={err_z:.2f}mm")
        else:
            print(f"  [----] {desc}  → {r.msg}")
        print()

    print("=" * 60)
    print("All tests passed." if all_pass else "SOME TESTS FAILED — check above.")
    sys.exit(0 if all_pass else 1)
