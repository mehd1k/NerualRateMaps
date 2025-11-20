#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import random
from message_filters import Subscriber, ApproximateTimeSynchronizer

class wall():
    def __init__(self, px, py, rx, ry) -> None:
        self.px = px
        self.py = py
        self.rx = rx
        self.ry = ry

class SynchronizerNode(Node):

    def __init__(self):
        super().__init__('synchronizer_node')

        self.image_sub = Subscriber(self, Image, '/my_camera/image_raw')
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')

        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.bridge = CvBridge()
        self.image_data = None
        self.lidar_data = None

        self.publisher_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        w1 = wall(-1.7, 1, 4, 0.4)
        w2 = wall(-3.4, 1.03, 0.4, 3.3)
        w3 = wall(-1.7, -3.2, 0.4, 2.9)
        w4 = wall(-1.7, -3.2, 3.8, 0.4)
        w5 = wall(-0.4, -5.5, 0.4, 0.7)
        self.wall_ls = [w1, w2, w3, w4, w5]
        self.x_lower = -0.6
        self.x_upper = 0.6
        self.y_lower = -0.6
        self.y_upper = 0.6
        self.counter = 0

        # Set the desired frequency (in Hz)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.process_callback)

    def sync_callback(self, image_msg, lidar_msg):
        self.image_data = image_msg
        self.lidar_data = lidar_msg

    def process_callback(self):
        if self.image_data is not None and self.lidar_data is not None:
            sec = self.image_data.header.stamp.sec
            nanosec = self.image_data.header.stamp.nanosec

            # Save image
            cv_image = self.bridge.imgmsg_to_cv2(self.image_data, desired_encoding='passthrough')
            image_filename = f'/home/mehdi/data_gazebo/image/img{sec}.{str(nanosec)[0]}.jpg'
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f'Image saved: {image_filename}')

            # Save lidar data
            lidar_filename = f'/home/mehdi/data_gazebo/scan_data/range{sec}.{str(self.lidar_data.header.stamp.nanosec)[0]}.npy'
            ranges = np.array(self.lidar_data.ranges)
            ranges[np.isinf(ranges)] = 2 * self.lidar_data.range_max
            np.save(lidar_filename, ranges)
            self.get_logger().info(f'Lidar data saved: {lidar_filename}')

            # Publish new position
            self.publish_new_position()

            # Reset data after processing
            self.image_data = None
            self.lidar_data = None

    def publish_new_position(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = "footprint_link"
        msg.joint_names = ["x_pose", "y_pose", "rot_joint"]

        point = JointTrajectoryPoint()
        x_pos, y_pos, orientation = self.new_state()
        point.positions = [x_pos, y_pos, orientation]
        point.time_from_start.sec = 1  # Change as needed

        msg.points = [point]
        self.publisher_.publish(msg)
        self.counter += 1
        self.get_logger().info('Publishing joint trajectory, counter = ' + str(self.counter))

    def new_state(self):
        orientation = random.uniform(-np.pi, np.pi)
        while True:
            x = random.uniform(self.x_lower, self.x_upper)
            y = random.uniform(self.y_lower, self.y_upper)
            if self.check_not_inside_walls(x, y):
                break

        return x, y, orientation

    def check_not_inside_walls(self, x, y):
        out = True
        for wall in self.wall_ls:
            if wall.px < x < wall.px + wall.rx and wall.py < y < wall.py + wall.ry:
                out = False
                return out
        return out

def main(args=None):
    rclpy.init(args=args)
    synchronizer_node = SynchronizerNode()
    rclpy.spin(synchronizer_node)
    synchronizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
