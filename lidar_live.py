#!/usr/bin/env python3
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanBuffer(Node):
    def __init__(self) -> None:
        super().__init__("plot_lidar_scan_live")

        self.declare_parameter("topic", "/scan")
        self.declare_parameter("point_size", 3.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value

        self._lock = threading.Lock()
        self._latest_xy: Optional[np.ndarray] = None  # shape (N, 2)
        self._latest_rmax: Optional[float] = None

        self._sub = self.create_subscription(LaserScan, topic, self._on_scan, 10)
        self.get_logger().info(f"Subscribing to LaserScan on '{topic}'")

    def _on_scan(self, msg: LaserScan) -> None:
        angle = msg.angle_min
        xs = []
        ys = []

        rmin = msg.range_min
        # Use the current scan's max valid range (from data), falling back to msg.range_max.
        data_rmax = 0.0
        for r in msg.ranges:
            if r >= rmin and math.isfinite(r):
                if r > data_rmax:
                    data_rmax = r
        rmax = data_rmax if data_rmax > 0.0 else float(msg.range_max)

        for r in msg.ranges:
            if rmin <= r <= rmax and math.isfinite(r):
                xs.append(r * math.cos(angle))
                ys.append(r * math.sin(angle))
            angle += msg.angle_increment

        xy = np.column_stack([xs, ys]) if xs else np.zeros((0, 2), dtype=np.float32)
       
        with self._lock:
            self._latest_xy = xy
            self._latest_rmax = float(rmax) if math.isfinite(rmax) else None

    def latest_xy(self) -> np.ndarray:
        with self._lock:
            if self._latest_xy is None:
                return np.zeros((0, 2), dtype=np.float32)
            return self._latest_xy.copy()

    def latest_rmax(self) -> Optional[float]:
        with self._lock:
            return self._latest_rmax

    def plot_params(self):
        point_size = self.get_parameter("point_size").get_parameter_value().double_value
        return float(point_size)


def main() -> None:
    rclpy.init()
    node = LaserScanBuffer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    point_size = node.plot_params()

    fig, ax = plt.subplots()
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Live LaserScan (XY)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)

    (scat,) = ax.plot([], [], "k.", markersize=point_size)
    (robot_dot,) = ax.plot([0.0], [0.0], "ro", markersize=4)

    def update(_frame):
        xy = node.latest_xy()
        rmax = node.latest_rmax()

        if xy.size == 0:
            scat.set_data([], [])
        else:
            scat.set_data(xy[:, 0], xy[:, 1])

        if rmax is not None and rmax > 0.0:
            # Add a small margin so points aren't glued to the border.
            lim = max(0.5, 1.05 * rmax)
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
        return scat, robot_dot

    ani = FuncAnimation(fig, update, interval=50, blit=True)

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
