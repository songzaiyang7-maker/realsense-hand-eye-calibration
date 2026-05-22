"""
Hand-Eye Calibration — Camera Data Collection (Windows + RealSense)

Detects chessboard corners, records camera pose via solvePnP on key press,
and saves results to JSON. Optionally sends a ZMQ trigger so the arm side
records simultaneously.

When used with record_arm.py, also receives real-time arm pose (rx/ry/rz)
and displays it on the camera feed to help adjust wrist orientations.

The chessboard pattern (inner corner count) and square size can be adjusted
via --pattern and --square to match your calibration board.

Usage:
    python collect_camera.py

    # Custom chessboard (9x6 inner corners, 20mm squares)
    python collect_camera.py --pattern 9 6 --square 20

    # With ZMQ trigger + arm angle display
    python collect_camera.py --zmq-port 5558 --arm-port 5559

Controls:
    SPACE  — Record current frame (when chessboard detected)
    Q/Esc  — Quit

Output:
    camera_data.json  — Camera intrinsics + per-frame poses
    calib_img_XX.jpg  — Saved images for each recorded frame
"""
import json
import time
import sys
import threading
import numpy as np
import cv2
import zmq
import pyrealsense2 as rs


def parse_args():
    args = {}
    argv = sys.argv[1:]
    for i, arg in enumerate(argv):
        if arg == '--pattern' and i + 2 < len(argv):
            args['pattern'] = (int(argv[i+1]), int(argv[i+2]))
        elif arg == '--square' and i + 1 < len(argv):
            args['square'] = float(argv[i+1])
        elif arg == '--zmq-port' and i + 1 < len(argv):
            args['zmq_port'] = int(argv[i+1])
        elif arg == '--arm-port' and i + 1 < len(argv):
            args['arm_port'] = int(argv[i+1])
    args.setdefault('pattern', (4, 6))
    args.setdefault('square', 15.0)
    args.setdefault('zmq_port', None)
    args.setdefault('arm_port', None)
    return args


def main():
    opts = parse_args()
    pattern = opts['pattern']       # (cols, rows) of inner corners
    square_mm = opts['square']      # Square side length in mm
    zmq_port = opts['zmq_port']
    arm_port = opts['arm_port']

    # ZMQ trigger publisher (optional, for synchronized arm recording)
    pub = None
    ctx = None
    if zmq_port:
        ctx = zmq.Context()
        pub = ctx.socket(zmq.PUB)
        pub.bind(f"tcp://*:{zmq_port}")

    # ZMQ subscriber for arm pose feedback (optional, for angle display)
    arm_sub = None
    arm_ctx = None
    arm_pose = {}
    arm_lock = threading.Lock()

    if arm_port:
        arm_ctx = zmq.Context()
        arm_sub = arm_ctx.socket(zmq.PULL)
        arm_sub.bind(f"tcp://*:{arm_port}")
        arm_sub.setsockopt(zmq.RCVTIMEO, 100)

        def _recv_arm():
            while True:
                try:
                    msg_str = arm_sub.recv_string()
                    data = json.loads(msg_str)
                    with arm_lock:
                        arm_pose.update(data)
                except zmq.Again:
                    continue

        t = threading.Thread(target=_recv_arm, daemon=True)
        t.start()

    # RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(config)

    # Camera intrinsics
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                              [0, intr.fy, intr.ppy],
                              [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    print(f"[Camera] Resolution: {intr.width}x{intr.height}")
    print(f"[Camera] fx={intr.fx:.2f} fy={intr.fy:.2f} cx={intr.ppx:.2f} cy={intr.ppy:.2f}")
    print(f"[Board]  Pattern: {pattern[0]}x{pattern[1]} inner corners, square={square_mm}mm")
    print(f"         (Adjust via --pattern COLS ROWS --square MM)")
    if zmq_port:
        print(f"[ZMQ]    Trigger port: {zmq_port}")
    if arm_port:
        print(f"[ZMQ]    Arm feedback port: {arm_port}")
    print("\nControls: SPACE=record  Q=quit")
    print("Make sure the robot arm is stable before pressing SPACE.\n")

    # Chessboard object points (Z=0 plane)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm

    collected = []
    arm_history = {"rx": [], "ry": [], "rz": []}

    cv2.namedWindow('Calibration Collect', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Calibration Collect', 960, 540)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue

            img = np.asanyarray(color.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = img.shape[:2]

            # Detect chessboard
            ret, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            display = img.copy()
            status = "NO BOARD"
            corners_ref = None

            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_ref = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(display, pattern, corners_ref, ret)
                status = f"BOARD DETECTED ({len(collected)} recorded)"

            # Top status
            cv2.putText(display, status, (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if ret else (0, 0, 255), 2)

            # Arm angle display
            with arm_lock:
                pose = dict(arm_pose)

            if pose and "rx" in pose:
                rx, ry, rz = pose["rx"], pose["ry"], pose["rz"]

                # Draw angle values (left side, below status)
                y0 = 65
                cv2.putText(display, f"rx: {rx:+7.1f} deg", (20, y0),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(display, f"ry: {ry:+7.1f} deg", (20, y0 + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(display, f"rz: {rz:+7.1f} deg", (20, y0 + 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

                # Track range across collected samples
                if arm_history["rx"]:
                    rx_range = max(arm_history["rx"]) - min(arm_history["rx"])
                    ry_range = max(arm_history["ry"]) - min(arm_history["ry"])
                    rz_range = max(arm_history["rz"]) - min(arm_history["rz"])
                    n = len(arm_history["rx"])

                    # Range display with color coding
                    ry_color = (0, 255, 0) if ry_range >= 20 else (0, 165, 255) if ry_range >= 10 else (0, 0, 255)
                    cv2.putText(display, f"Range({n}): rx={rx_range:.0f} ry={ry_range:.0f} rz={rz_range:.0f} deg",
                                (20, y0 + 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, ry_color, 1)

                    if ry_range < 20:
                        cv2.putText(display, "ry < 20 deg! Tilt wrist more", (20, y0 + 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # Bottom info
            cv2.putText(display, f"Recorded: {len(collected)}   SPACE=save  Q=quit",
                        (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow('Calibration Collect', display)

            key = cv2.waitKey(50)

            if key == ord('q') or key == 27:
                break
            elif key == ord(' ') and ret:
                _, rvec, tvec = cv2.solvePnP(objp, corners_ref, camera_matrix, dist_coeffs)

                idx = len(collected) + 1
                entry = {
                    "idx": idx,
                    "rvec": rvec.flatten().tolist(),
                    "tvec": tvec.flatten().tolist(),
                    "timestamp": time.time(),
                }
                collected.append(entry)

                # Record arm angle range at this point
                if pose and "rx" in pose:
                    arm_history["rx"].append(pose["rx"])
                    arm_history["ry"].append(pose["ry"])
                    arm_history["rz"].append(pose["rz"])

                cv2.imwrite(f"calib_img_{idx:02d}.jpg", img)

                if pub:
                    pub.send_string(json.dumps({"trigger": idx, "timestamp": time.time()}))

                pose_str = ""
                if pose and "rx" in pose:
                    pose_str = f"  arm=({pose['rx']:.1f}, {pose['ry']:.1f}, {pose['rz']:.1f})"
                print(f"[{idx}] Recorded{pose_str}")

    finally:
        # Save collected data
        if collected:
            data = {
                "pattern": list(pattern),
                "square_mm": square_mm,
                "camera_matrix": camera_matrix.tolist(),
                "dist_coeffs": dist_coeffs.tolist(),
                "poses": collected,
            }
            path = "camera_data.json"
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"\nSaved {len(collected)} samples to {path}")
        else:
            print("\nNo data collected.")

        pipeline.stop()
        if pub:
            pub.close()
        if ctx:
            ctx.term()
        if arm_sub:
            arm_sub.close()
        if arm_ctx:
            arm_ctx.term()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
