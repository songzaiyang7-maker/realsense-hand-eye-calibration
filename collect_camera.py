"""
Hand-Eye Calibration — Camera Data Collection (Windows + RealSense)

Detects chessboard corners, records camera pose via solvePnP on key press,
and saves results to JSON. Optionally sends a ZMQ trigger so the arm side
records simultaneously.

The chessboard pattern (inner corner count) and square size can be adjusted
via --pattern and --square to match your calibration board.

Usage:
    python collect_camera.py

    # Custom chessboard (9x6 inner corners, 20mm squares)
    python collect_camera.py --pattern 9 6 --square 20

    # With ZMQ trigger for synchronized arm recording
    python collect_camera.py --zmq-port 5558

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
    args.setdefault('pattern', (4, 6))
    args.setdefault('square', 15.0)
    args.setdefault('zmq_port', None)
    return args


def main():
    opts = parse_args()
    pattern = opts['pattern']       # (cols, rows) of inner corners
    square_mm = opts['square']      # Square side length in mm
    zmq_port = opts['zmq_port']

    # ZMQ trigger publisher (optional, for synchronized arm recording)
    pub = None
    ctx = None
    if zmq_port:
        ctx = zmq.Context()
        pub = ctx.socket(zmq.PUB)
        pub.bind(f"tcp://*:{zmq_port}")

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
    print("\nControls: SPACE=record  Q=quit")
    print("Make sure the robot arm is stable before pressing SPACE.\n")

    # Chessboard object points (Z=0 plane)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm

    collected = []
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

            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_ref = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(display, pattern, corners_ref, ret)
                status = f"BOARD DETECTED ({len(collected)} recorded)"

            cv2.putText(display, status, (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if ret else (0, 0, 255), 2)
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

                cv2.imwrite(f"calib_img_{idx:02d}.jpg", img)

                if pub:
                    pub.send_string(json.dumps({"trigger": idx, "timestamp": time.time()}))

                print(f"[{idx}] Recorded (tvec={rvec.flatten()[:3]})")

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
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
