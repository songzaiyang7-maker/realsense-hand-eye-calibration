"""
Hand-Eye Calibration — Arm Pose Recorder (ROS2 / WSL2)

Subscribes to the robot arm's ToolVectorActual topic and records poses
when triggered by the camera-side ZMQ signal from collect_camera.py.

Requires:
    - ROS2 Humble
    - dobot_msgs_v4 package (or modify the topic/message type for your robot)
    - pyzmq

Usage:
    cd ~/my_robot_ws && source install/setup.bash
    ros2 run <your_pkg> record_arm --ros-args \
        -p windows_host:=172.23.224.1 -p zmq_port:=5558

Output:
    ~/arm_data.json  — Recorded arm poses matched to camera triggers

To adapt for a different robot:
    Change the subscription topic and message type in arm_callback().
    The arm data must provide: x, y, z (mm), rx, ry, rz (degrees).
"""
import json
import threading
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq

from dobot_msgs_v4.msg import ToolVectorActual


class CalibrationRecorder(Node):
    def __init__(self):
        super().__init__('calibration_recorder')

        self.declare_parameter('windows_host', '172.28.80.1')
        self.declare_parameter('zmq_port', 5558)

        host = self.get_parameter('windows_host').value
        port = self.get_parameter('zmq_port').value

        # Current arm pose
        self._arm_pose = {}

        # Subscribe to arm pose topic
        # Adapt the topic name and message type for your robot
        self.create_subscription(
            ToolVectorActual,
            '/dobot_msgs_v4/msg/ToolVectorActual',
            self.arm_callback, 10
        )

        self._collected = []

        # ZMQ subscriber (separate thread)
        self._running = True
        self._zmq_thread = threading.Thread(
            target=self._zmq_loop, args=(host, port), daemon=True
        )
        self._zmq_thread.start()

        self.create_timer(5.0, self._status)
        self.get_logger().info(f'Calibration recorder ready -> tcp://{host}:{port}')

    def arm_callback(self, msg: ToolVectorActual):
        """Store the latest arm pose. Adapt for your robot's message type."""
        self._arm_pose = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "rx": msg.rx,
            "ry": msg.ry,
            "rz": msg.rz,
        }

    def _zmq_loop(self, host, port):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.connect(f'tcp://{host}:{port}')
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.setsockopt(zmq.RCVTIMEO, 1000)

        while self._running:
            try:
                msg_str = sock.recv_string()
                data = json.loads(msg_str)
                idx = data.get('trigger')
                if idx is None:
                    continue

                if not self._arm_pose:
                    self.get_logger().warn(
                        f'Trigger #{idx} received but no arm pose yet!')
                    continue

                entry = {"idx": idx, **self._arm_pose}
                self._collected.append(entry)
                self.get_logger().info(
                    f'[{idx}] Recorded arm pose: '
                    f'({self._arm_pose["x"]:.1f}, '
                    f'{self._arm_pose["y"]:.1f}, '
                    f'{self._arm_pose["z"]:.1f})'
                )

            except zmq.Again:
                continue

        sock.close()
        ctx.term()

    def _status(self):
        if self._collected:
            self.get_logger().info(
                f'Arm poses recorded: {len(self._collected)}')

    def destroy_node(self):
        self._running = False
        self._zmq_thread.join(timeout=3)

        # Save collected data
        if self._collected:
            path = os.path.expanduser('~/arm_data.json')
            data = {
                "arm_type": "dobot",
                "pose_fields": ["x", "y", "z", "rx", "ry", "rz"],
                "poses": self._collected,
            }
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(
                f'Saved {len(self._collected)} poses to {path}')
        else:
            self.get_logger().warn('No arm poses collected')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
