#!/usr/bin/env python3
"""Report what each topic actually delivers.

Slide: 'Demo 3', the line 'Watch the publish rate. It is not the rate you
configured, and it drops as the scene gets busy.'

`ros2 topic hz` gives you one topic at a time. This prints all five side by
side, along with the gap between the newest and oldest stamp in the set, which
is the number that matters before you fuse anything: two measurements you
combine must describe the same instant, and these do not.

    ros2 run l2_carla_demo rate_report
"""

import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2

TOPICS = [
    ("camera", Image, "/carla/ego_vehicle/rgb_front/image"),
    ("lidar", PointCloud2, "/carla/ego_vehicle/lidar"),
    ("radar", PointCloud2, "/carla/ego_vehicle/radar_front"),
    ("imu", Imu, "/carla/ego_vehicle/imu"),
    ("gnss", NavSatFix, "/carla/ego_vehicle/gnss"),
]

WINDOW = 5.0        # seconds of history each rate is measured over


class RateReport(Node):

    def __init__(self) -> None:
        super().__init__("rate_report")
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.stamps: dict[str, collections.deque] = {}
        self.latest: dict[str, float] = {}

        for name, msg_type, topic in TOPICS:
            self.stamps[name] = collections.deque()
            self.create_subscription(
                msg_type, topic,
                lambda msg, n=name: self._record(n, msg), qos)

        self.create_timer(2.0, self._report)
        self.get_logger().info("listening; first report in 2 s")

    def _record(self, name: str, msg) -> None:
        # The message stamp, not the arrival time: in synchronous mode the
        # simulator owns the clock, and arrival time measures your network.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest[name] = stamp
        window = self.stamps[name]
        window.append(stamp)
        while window and stamp - window[0] > WINDOW:
            window.popleft()

    def _report(self) -> None:
        print()
        print(f"{'topic':<8} {'rate':>9}   {'messages':>8}")
        print("-" * 30)
        for name, _, _ in TOPICS:
            window = self.stamps[name]
            if len(window) < 2:
                print(f"{name:<8} {'no data':>9}   {len(window):>8}")
                continue
            span = window[-1] - window[0]
            rate = (len(window) - 1) / span if span > 0 else float("nan")
            print(f"{name:<8} {rate:8.1f} Hz {len(window):>8}")

        if len(self.latest) > 1:
            spread = max(self.latest.values()) - min(self.latest.values())
            print(f"\nnewest stamp minus oldest: {spread * 1000:.1f} ms")
            print("At 30 km/h the vehicle covers about "
                  f"{spread * 8.33 * 100:.1f} cm in that gap.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RateReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
