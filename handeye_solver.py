"""
Hand-Eye Calibration Solver (Eye-in-Hand)

Reads camera poses (from chessboard detection) + arm poses (from robot controller),
solves AX=XB for the camera-to-end-effector transform using multiple methods,
auto-selects the best result, and verifies correctness.

Coordinate convention (eye-in-hand):
    A * X = X * B
    A = target_to_camera  (from solvePnP)
    B = gripper_to_base   (from robot)
    X = camera_to_gripper (the unknown hand-eye matrix)

Usage:
    python handeye_solver.py --camera camera_data.json --arm arm_data.json

Output:
    handeye_result.json  — camera-to-end-effector transform

Options:
    --exclude 4 7        Exclude specific sample indices (outliers)
    --euler zyx          Euler angle convention (default: zyx for Dobot CR5)
                        Supported: zyx (Rz@Ry@Rx), xyz (Rx@Ry@Rz), zyz
    --out result.json    Output file path
"""
import json
import argparse
import numpy as np
import cv2


# ── Euler angle conversions ──────────────────────────────

def euler_to_rotmat(rx, ry, rz, convention="zyx"):
    """Convert Euler angles (degrees) to rotation matrix.

    Supported conventions:
        zyx  — Rz @ Ry @ Rx  (default, Dobot CR5)
        xyz  — Rx @ Ry @ Rz
        zyz  — Rz @ Ry @ Rz
    """
    rx, ry, rz = np.radians([rx, ry, rz])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0,           1, 0          ],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,           0,           1]])

    if convention == "zyx":
        return Rz @ Ry @ Rx
    elif convention == "xyz":
        return Rx @ Ry @ Rz
    elif convention == "zyz":
        return Rz @ Ry @ Rz
    else:
        raise ValueError(f"Unknown euler convention: {convention}")


def rotmat_to_euler(R, convention="zyx"):
    """Extract Euler angles (degrees) from rotation matrix."""
    if convention == "zyx":
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
            ry = np.degrees(np.arctan2(-R[2, 0], sy))
            rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        else:
            rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
            ry = np.degrees(np.arctan2(-R[2, 0], sy))
            rz = 0
    else:
        # Fallback for other conventions
        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        ry = np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2)))
        rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return rx, ry, rz


# ── Rotation variation analysis ──────────────────────────

def rotation_variation(rot_mats):
    """Evaluate whether rotation changes are sufficient for calibration.

    Computes the condition number of the rotation-axis covariance matrix.
    A higher value (>3.0) indicates diverse rotations, which is desirable.
    """
    axes = []
    for i in range(1, min(len(rot_mats), 10)):
        R_rel = rot_mats[i].T @ rot_mats[0]
        angle = np.arccos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1))
        if np.sin(angle) > 1e-6:
            axis = np.array([R_rel[2, 1] - R_rel[1, 2],
                             R_rel[0, 2] - R_rel[2, 0],
                             R_rel[1, 0] - R_rel[0, 1]]) / (2 * np.sin(angle))
            axes.append(axis)

    if len(axes) < 2:
        return 0.0

    axes = np.array(axes)
    cov = np.cov(axes.T)
    eigenvalues = np.linalg.eigvalsh(cov)
    return eigenvalues[2] / (eigenvalues[0] + 1e-10)


# ── Main solver ──────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Hand-Eye Calibration Solver (Eye-in-Hand)")
    parser.add_argument('--camera', default='camera_data.json',
                        help='Camera data JSON from collect_camera.py')
    parser.add_argument('--arm', default='arm_data.json',
                        help='Arm data JSON from record_arm.py')
    parser.add_argument('--out', default='handeye_result.json',
                        help='Output path for calibration result')
    parser.add_argument('--exclude', nargs='+', type=int, default=[],
                        help='Sample indices to exclude (e.g. --exclude 4 7)')
    parser.add_argument('--euler', default='zyx',
                        choices=['zyx', 'xyz', 'zyz'],
                        help='Euler angle convention (default: zyx)')
    args = parser.parse_args()

    # Load data
    with open(args.camera) as f:
        cam_data = json.load(f)
    with open(args.arm) as f:
        arm_data = json.load(f)

    cam_poses = {p["idx"]: p for p in cam_data["poses"]}
    arm_poses = {p["idx"]: p for p in arm_data["poses"]}

    # Match by index
    common = sorted(set(cam_poses) & set(arm_poses))
    print(f"Camera samples: {len(cam_poses)}")
    print(f"Arm samples:    {len(arm_poses)}")
    print(f"Matched:        {len(common)}")

    # Exclude outliers
    if args.exclude:
        common = [i for i in common if i not in args.exclude]
        print(f"Excluded idx={args.exclude}, remaining: {len(common)}")

    if len(common) < 5:
        print("ERROR: Too few matched samples (need >= 5). Recollect data.")
        return

    # Analyze arm angle variation
    rx_list = [arm_poses[i]["rx"] for i in common]
    ry_list = [arm_poses[i]["ry"] for i in common]
    rz_list = [arm_poses[i]["rz"] for i in common]

    print("--- Arm angle variation analysis ---")
    print(f"  rx: {min(rx_list):.1f} ~ {max(rx_list):.1f}  (range {max(rx_list)-min(rx_list):.1f} deg)")
    print(f"  ry: {min(ry_list):.1f} ~ {max(ry_list):.1f}  (range {max(ry_list)-min(ry_list):.1f} deg)")
    print(f"  rz: {min(rz_list):.1f} ~ {max(rz_list):.1f}  (range {max(rz_list)-min(rz_list):.1f} deg)")

    ry_range = max(ry_list) - min(ry_list)

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for idx in common:
        cp = cam_poses[idx]
        ap = arm_poses[idx]

        # Camera: solvePnP output = target->camera
        rvec = np.array(cp["rvec"], dtype=np.float64).reshape(3, 1)
        tvec = np.array(cp["tvec"], dtype=np.float64).reshape(3, 1) / 1000.0
        R_target2cam.append(cv2.Rodrigues(rvec)[0])
        t_target2cam.append(tvec)

        # Arm: x/y/z(mm) + rx/ry/rz(deg) -> gripper->base
        t = np.array([[ap["x"] / 1000.0],
                       [ap["y"] / 1000.0],
                       [ap["z"] / 1000.0]], dtype=np.float64)
        R = euler_to_rotmat(ap["rx"], ap["ry"], ap["rz"], args.euler)
        R_gripper2base.append(R)
        t_gripper2base.append(t)

    # Evaluate rotation diversity
    variation = rotation_variation(R_gripper2base)
    print(f"\nRotation variation score: {variation:.2f} (need > 3.0)")
    if ry_range < 20:
        print(f"WARNING: ry variation only {ry_range:.1f} deg (need >= 20~30 deg)")
        print("  Tilt the wrist up/down more during data collection.")
    if variation < 3.0:
        print("WARNING: Low rotation diversity, result may be unreliable.")
        print("  Recollect with more diverse wrist orientations.")

    # Solve with all 4 methods
    methods = [
        ("Tsai-Lenz",   cv2.CALIB_HAND_EYE_TSAI),
        ("Park",        cv2.CALIB_HAND_EYE_PARK),
        ("Horaud",      cv2.CALIB_HAND_EYE_HORAUD),
        ("Daniilidis",  cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]

    best_R, best_t = None, None
    best_err = float('inf')
    best_method = ""
    all_results = []

    for name, method in methods:
        try:
            R, t = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method
            )

            H = np.eye(4)
            H[:3, :3] = R
            H[:3, 3] = t.flatten()

            # Verification: gripper2base @ cam2gripper @ target2cam = target2base (should be constant)
            world_poses = []
            for idx in common:
                cp = cam_poses[idx]
                ap = arm_poses[idx]
                R_t2c = cv2.Rodrigues(np.array(cp["rvec"]))[0]
                t_t2c = np.array(cp["tvec"]).reshape(3, 1) / 1000.0
                R_g2b = euler_to_rotmat(ap["rx"], ap["ry"], ap["rz"], args.euler)
                t_g2b = np.array([[ap["x"]/1000], [ap["y"]/1000], [ap["z"]/1000]])

                A = np.eye(4); A[:3,:3] = R_t2c; A[:3,3] = t_t2c.flatten()
                B = np.eye(4); B[:3,:3] = R_g2b; B[:3,3] = t_g2b.flatten()
                world_poses.append((B @ H @ A, idx))

            ref = world_poses[0][0]
            errs = [np.linalg.norm(pose - ref) for pose, _ in world_poses]
            mean_err = np.mean(errs)
            std = np.std([pose[:3,3] for pose, _ in world_poses], axis=0) * 1000
            t_mm = t.flatten() * 1000

            print(f"\n{name:15s}  err={mean_err:.4f}  "
                  f"t=({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f})  "
                  f"std=({std[0]:.1f}, {std[1]:.1f}, {std[2]:.1f})mm")

            all_results.append({
                "method": name, "R": R, "t": t,
                "error": mean_err, "std": std,
            })

            if mean_err < best_err:
                best_err = mean_err
                best_R, best_t = R, t
                best_method = name

        except cv2.error as e:
            print(f"\n{name:15s}  FAILED: {str(e)[:80]}")

    if best_R is None:
        print("\nERROR: All methods failed!")
        print("Suggestions:")
        print("  1. Ensure wrist rotates with diverse orientations (ry variation > 30 deg)")
        print("  2. Check idx matching between camera and arm data")
        print("  3. Collect 10~15 valid samples")
        return

    # Report consistency across methods
    if len(all_results) > 1:
        translations = [r["t"].flatten() * 1000 for r in all_results]
        t_std = np.std(translations, axis=0)
        print(f"\nCross-method translation std: ({t_std[0]:.1f}, {t_std[1]:.1f}, {t_std[2]:.1f})mm")

    # Final result
    R_cam2gripper, t_cam2gripper = best_R, best_t
    H = np.eye(4)
    H[:3, :3] = R_cam2gripper
    H[:3, 3] = t_cam2gripper.flatten()

    rx, ry, rz = rotmat_to_euler(R_cam2gripper, args.euler)

    print("\n" + "=" * 60)
    print(f"Hand-Eye Result (camera -> end_effector)  [{best_method}]")
    print("=" * 60)
    print(f"\nRotation matrix R:")
    print(np.round(R_cam2gripper, 4))
    print(f"\nTranslation t (m):")
    print(np.round(t_cam2gripper.flatten(), 4))
    print(f"\nEuler angles (deg): rx={rx:.2f}  ry={ry:.2f}  rz={rz:.2f}")
    print(f"Translation (mm): {t_cam2gripper.flatten() * 1000}")
    print(f"\nHomogeneous matrix (4x4):")
    print(np.round(H, 4))
    print("=" * 60)

    if best_err > 1.0:
        print(f"\nWARNING: Error {best_err:.4f} is too large, calibration accuracy insufficient.")
    elif best_err > 0.1:
        print(f"\nOK: Error {best_err:.4f} is acceptable.")
    else:
        print(f"\nGOOD: Error {best_err:.4f} is excellent!")

    # Verification details
    print("\nVerification (checkerboard world coordinates should be constant):")
    world_poses = []
    for idx in common:
        cp = cam_poses[idx]
        ap = arm_poses[idx]
        R_t2c = cv2.Rodrigues(np.array(cp["rvec"]))[0]
        t_t2c = np.array(cp["tvec"]).reshape(3, 1) / 1000.0
        R_g2b = euler_to_rotmat(ap["rx"], ap["ry"], ap["rz"], args.euler)
        t_g2b = np.array([[ap["x"]/1000], [ap["y"]/1000], [ap["z"]/1000]])

        A = np.eye(4); A[:3,:3] = R_t2c; A[:3,3] = t_t2c.flatten()
        B = np.eye(4); B[:3,:3] = R_g2b; B[:3,3] = t_g2b.flatten()
        T_world = B @ H @ A
        world_poses.append((T_world, idx))

    ref_pose = world_poses[0][0]
    errors = []
    for T_world, idx in world_poses:
        err = np.linalg.norm(T_world - ref_pose)
        pos = T_world[:3,3] * 1000
        errors.append(err)
        print(f"  [{idx}] target2base=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})mm  err={err:.4f}")

    std = np.std([w[:3,3] for w, _ in world_poses], axis=0) * 1000
    print(f"\n  Translation std: ({std[0]:.1f}, {std[1]:.1f}, {std[2]:.1f}) mm")

    # Save result
    result = {
        "method": best_method,
        "euler_convention": args.euler,
        "R_cam2gripper": np.round(R_cam2gripper, 6).tolist(),
        "t_cam2gripper_m": np.round(t_cam2gripper.flatten(), 6).tolist(),
        "euler_deg": {"rx": round(rx, 2), "ry": round(ry, 2), "rz": round(rz, 2)},
        "translation_mm": np.round(t_cam2gripper.flatten() * 1000, 2).tolist(),
        "matrix_4x4": np.round(H, 6).tolist(),
        "verification_errors": [round(e, 4) for e in errors],
        "translation_std_mm": np.round(std, 2).tolist(),
    }
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"\nResult saved to {args.out}")


if __name__ == '__main__':
    main()
