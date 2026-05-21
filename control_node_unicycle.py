#!/usr/bin/env python3

"""
Gazebo Neural Analysis Node
Created for Neural Rate Maps project
Integrates Gazebo camera data with MATLAB neural analysis pipeline
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time
import matlab.engine
from gen_controller import cell_ls
from find_controller_orientation import control_gain_load
import argparse
import json
from datetime import datetime
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from cell_configs import cell_ls
from utils_ros import gen_grid_points
from vaemodel import load_vae_model
from gen_controller import vectorize_matrix, load_RSC_data
from plot_vector_field_from_lidar import generate_occupancy_grid_scan
import time
from nav_msgs.msg import Odometry
# Import torch for VAE model loading
try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. VAE functionality will be disabled.")
def quaternion_to_yaw(q):
    """Extract yaw (heading) in radians from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def compress_highlights(img_bgr):
    # img_bgr in [0,1] float32 preferred
    gamma = 1.4                  # 1.2–1.8 darkens highlights; tune
    img = np.clip(img_bgr, 0, 1)
    return np.power(img, gamma)  # simple tone curve

def reinhard_tonemap(img_bgr):
    img = np.clip(img_bgr, 0, 1)
    return img / (1.0 + img)     # compresses bright regions more


def gray_clip(img_bgr, thresh=0.6, gray_value=0.4, blur_px=0.5, blend_width=0.01):
    """
    Replace bright pixels with a gray tone.
    img_bgr: np.ndarray (float32 or uint8)
    thresh: brightness threshold (0–1 for float, 0–255 for uint8)
    gray_value: target gray value (same scale as image)
    """
    img = img_bgr.astype(np.float32)
    if img.max() > 1.5:  # if it's uint8, normalize first
        img /= 255.0

    # Convert to grayscale to measure brightness
    gray = img

    # Create mask of bright regions
    mask = gray > thresh

    # Option 1: map those pixels to gray_value directly
    # img[mask] = gray_value
    # Smooth mask: ramp up from (thresh - blend_width) to thresh
    alpha = np.clip((gray - (thresh - blend_width)) / blend_width, 0, 1)

    # Optional: blur the alpha to make transitions smooth
    if blur_px > 0:
        alpha = cv2.GaussianBlur(alpha, (0, 0), blur_px)

    # Blend gray tone with original image
    if img.ndim == 2:
        img = img * (1 - alpha) + gray_value * alpha
    else:
        img = img * (1 - alpha[..., None]) + gray_value * alpha[..., None]

    return np.clip(img, 0, 1)




def generate_occupancy_grid_polar(lidar_scan, num_angle_bins=120, num_range_bins=40):
    """
    Generate an occupancy grid in polar coordinates based on lidar scan data.
    
    Args:
        json_file: Path to JSON file containing lidar scan data
        num_angle_bins: Number of angle bins (default: 120, corresponding to 3-degree increments)
        num_range_bins: Number of range bins (default: 40)
    
    Returns:
        occupancy_grid: 2D numpy array of shape (num_range_bins, num_angle_bins) with values:
            - 1.0: occupied cell (contains obstacles)
            - 0.0: free cell (along ray path, no obstacles)
            - 0.5: unknown cell (not explored)
        polar_params: Dictionary with 'angle_min', 'angle_max', 'angle_increment', 
                     'range_min', 'range_max', 'range_increment' defining the grid parameters
    """
    # Load data
  
    
    # Extract lidar parameters
    ranges = np.array(lidar_scan['ranges'])
    angle_min = lidar_scan['angle_min']
    angle_max = lidar_scan['angle_max']
    angle_increment = lidar_scan['angle_increment']
    range_min = lidar_scan['range_min']
    range_max = lidar_scan['range_max']
    
    # Generate angles for lidar data
    num_points = len(ranges)
    if angle_increment > 0:
        angles = np.arange(angle_min, angle_min + num_points * angle_increment, angle_increment)
        angles = angles[:num_points]
    else:
        angles = np.linspace(angle_min, angle_max, num_points)
    
    # Filter out invalid ranges
    valid_mask = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    ranges_valid = ranges[valid_mask]
    angles_valid = angles[valid_mask]
    
    # Define polar grid parameters
    # Angle bins: 120 bins with 3-degree increments covering full 360 degrees
    angle_bin_increment = 3.0 * np.pi / 180.0  # 3 degrees in radians
    angle_grid_min = 0.0  # Start from 0 radians (East)
    angle_grid_max = num_angle_bins * angle_bin_increment  # Full circle
    
    # Range bins: 40 bins from range_min to range_max
    # range_increment = (range_max - range_min) / num_range_bins
    range_increment = 1.2 / num_range_bins
    
    # Initialize occupancy grid (0.5 = unknown)
    occupancy_grid = np.full((num_range_bins, num_angle_bins), 0.0, dtype=np.float32)
    
    # Function to convert (angle, range) to grid indices
    def polar_to_grid(angle, range_val):
        """Convert polar coordinates to grid indices."""
        # Normalize angle to [0, 2π) range
        angle_norm = angle % (2 * np.pi)
        if angle_norm < 0:
            angle_norm += 2 * np.pi
        
        # Find angle bin index
        angle_bin = int(angle_norm / angle_bin_increment)
        angle_bin = np.clip(angle_bin, 0, num_angle_bins - 1)
        
        # Find range bin index
        range_bin = int((range_val - range_min) / range_increment)
        range_bin = np.clip(range_bin, 0, num_range_bins - 1)
        
        return range_bin, angle_bin
    
    # Mark occupied cells (cells containing obstacle endpoints)
    for angle, range_val in zip(angles_valid, ranges_valid):
        range_bin, angle_bin = polar_to_grid(angle, range_val)
        occupancy_grid[range_bin, angle_bin] = 1.0  # Occupied
    
    # Mark free cells (cells along ray path from robot to obstacle)
    for angle, range_val in zip(angles_valid, ranges_valid):
        if range_val > 0:
            # Get grid position of obstacle
            end_range_bin, end_angle_bin = polar_to_grid(angle, range_val)
            
            # Mark all range bins from 0 to obstacle as free
            # (except the obstacle cell itself which is already marked as occupied)
            for r_bin in range(end_range_bin):
                # Keep obstacle cells as occupied, mark others as free
                if occupancy_grid[r_bin, end_angle_bin] != 1.0:
                    occupancy_grid[r_bin, end_angle_bin] = 0.0  # Free
    
    # Store polar parameters for reference
    polar_params = {
        'angle_min': angle_grid_min,
        'angle_max': angle_grid_max,
        'angle_increment': angle_bin_increment,
        'num_angle_bins': num_angle_bins,
        'range_min': range_min,
        'range_max': range_max,
        'range_increment': range_increment,
        'num_range_bins': num_range_bins
    }
    
    return occupancy_grid, polar_params

class GazeboNeuralAnalysisNode(Node):
    """
    ROS node that subscribes to Gazebo camera data and processes it through 
    the MATLAB neural analysis pipeline similar to the original Panda3D implementation.
    """
    
    def __init__(self, mode, cell_id_list=None, heading_list=None):
        super().__init__('gazebo_neural_analysis_node')
         
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Image processing parameters (matching original Panda3D implementation)
        self.VX, self.VY = 150, 90  # Target image size
        self.sc_img = 1
        self.cam_far = int(3000/self.sc_img)
        # measurement mode = ['neural_rate', 'neural_lidar', 'vae']
        self.measurement_mode = 'neural_lidar'
        # Initialize control system components
        self.cell_ls = cell_ls
        self.control_gain_load = control_gain_load(self.measurement_mode)
        self.dt = 0.01
        self.cell_id_list = [int(cell_id) for cell_id in (cell_id_list if cell_id_list is not None else [0, 1])]
        if not self.cell_id_list:
            raise ValueError("cell_id_list must contain at least one cell identifier")
        heading_list = heading_list if heading_list is not None else [0.0]
        if not heading_list:
            raise ValueError("heading_list must contain at least one heading angle")
        self.heading_list = [float(h) for h in heading_list]
        self.current_cell_id = None
        self.current_grid_occ = None
        time.sleep(2)
        self.get_logger().info("Waiting for 2 seconds to start the node")
        # # Robot state
        # self.current_position = np.array([0.22, 0.7])  # Initial position
        # self.current_hd = 270  #Initial heading direction



        self.current_position = np.array([1.0, 0.2])  # Initial position
        self.current_hd = 90  #Initial heading direction

        self.current_step = 0
        self.num_steps = 300
        self.RSC_data = load_RSC_data()
        # self.current_position = self.current_position - self.bias_position
        

        self.cell_ls = cell_ls
        # Data storage
        self.image_ls = []
        self.postion_ls = []
        self.hd_ls = []
        self.ratemap_ls = []
        self.u_ls = []
        self.v_ls = []
        self.omega_ls = []
        self.odom_ls = []
        
        # State tracking for gen_data mode
        self.pose_published = False  # Track if we've published pose for current step
        self.image_timestamp = None  # Track when last image was received
        self.current_scan = None  # Track current lidar scan data
        self.should_shutdown = False  # Flag to signal completion
        
        # Create subscribers
        self.image_sub = Subscriber(self, Image, '/my_camera/image_raw')
        # Use sensor data QoS for lidar (best effort, small queue for real-time data)
        scan_qos = qos_profile_sensor_data
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/demo/scan', 
            self.scan_callback, 
            scan_qos
        )
        self.get_logger().info("Subscribed to /scan topic for lidar data with sensor QoS profile")
        
        # Create synchronizer for image and pose data
        self.ts = ApproximateTimeSynchronizer([self.image_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        # Create publishers for robot control
        ##linear model 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_linear', 10)
        # self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        # self.publisher_pose = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)



        # edited by melinda (unicycle model)
        # self._scan_sub = self.create_subscription(
        #     LaserScan, "/demo/scan", self._scan_cb, 10,
        # )
        self._pose_sub = self.create_subscription(
            #PoseStamped, "/vrpn_client_node/jackal/pose", self._pose_cb, 10,
            Odometry, "/demo/odom_demo", self._pose_cb, 10,
        )
        # edited by melinda
        self._control_pub = self.create_publisher(
            Twist, "/demo/cmd_demo", 10
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for processing
        self.timer_period = 0.1  # 10 Hz


    
        
        
        self.timer = self.create_timer(self.timer_period, self.process_callback)
       



     



        # Create output directories
        self.create_output_directories()
        
        # JSON data logging
        self.json_data_dir = 'test/data'
        self.json_data_list = []





        ##test
        # self.vector_field_collected_data(9, 270)
        # sys.exit()
        if self.measurement_mode == 'neural_rate':
            # Initialize MATLAB engine
            self.get_logger().info("Starting MATLAB engine...")
            try:
                self.eng = matlab.engine.start_matlab()
                self.get_logger().info("MATLAB engine started successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to start MATLAB engine: {e}")
                sys.exit(1)
            
        
        self.get_logger().info("Gazebo Neural Analysis Node initialized successfully")
        
        # Check if scan topic is available (async check after a short delay, run once)
        self.scan_check_done = False
        # self.create_timer(1.0, self.check_scan_topic)


    def _pose_cb(self, msg):
        self._latest_pose = msg
        # added extra pose because odometry messages wrap the pose inside another layer 
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        self.current_position = np.array([p.x, p.y]).reshape(2, 1)
        self.current_velocity = np.array([vx, vy, omega]).reshape(3, 1)
        self.get_logger().info(f"current velocity: {vx:.3f}, {vy:.3f}, {omega:.3f}")
        self.current_hd = quaternion_to_yaw(q)
        self.current_hd_degree = self.current_hd * 180 / np.pi % 360
        self.get_logger().info(
            "pose: x=%.3f y=%.3f yaw=%.3f degree" % (p.x, p.y, self.current_hd_degree),
            throttle_duration_sec=1.0,
        )
        self.current_cell = self.find_cell(self.current_position)
        self.get_logger().info(f"current cell: {self.current_cell}")
    
    
        if self.current_cell is not None:
            self.get_logger().info(
                "current cell: %d" % (self.current_cell),
                throttle_duration_sec=1.0,
            )
        # else:
        #     self.get_logger().info(
        #         "current cell: None",
        #         throttle_duration_sec=1.0,)

   
    
    def create_output_directories(self):
        """Create necessary output directories"""
        directories = ['trj', 'test/img', 'test/data']
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
            self.get_logger().info(f"Created directory: {directory}")
    
    def image_callback(self, image_msg):
        """Callback for synchronized image data"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
            
            # Process the image to match original format
            processed_image = self.process_image(cv_image)
            
            # Apply Gaussian blur to reduce noise
            processed_image = cv2.GaussianBlur(processed_image, (5, 5), 0)
            
            # processed_image = compress_highlights(processed_image)
            # processed_image = reinhard_tonemap(processed_image)
            # processed_image = gray_clip(processed_image)
            # # Store the processed image
            self.current_image = processed_image
            
            # Track when image was received
            import time
            self.image_timestamp = time.time()
            
            self.get_logger().debug(f"Processed image with shape: {processed_image.shape}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    
    
    def scan_callback(self, scan_msg):
        """Callback for lidar scan data"""
        # self.get_logger().info(f"[SCAN] Callback triggered! Received scan message with {len(scan_msg.ranges)} ranges")
        try:
            # Store the scan data
            self.current_scan = {
                "ranges": list(scan_msg.ranges),
                "intensities": list(scan_msg.intensities) if len(scan_msg.intensities) > 0 else [],
                "angle_min": float(scan_msg.angle_min),
                "angle_max": float(scan_msg.angle_max),
                "angle_increment": float(scan_msg.angle_increment),
                "time_increment": float(scan_msg.time_increment),
                "scan_time": float(scan_msg.scan_time),
                "range_min": float(scan_msg.range_min),
                "range_max": float(scan_msg.range_max)
            }
            self.current_grid_occ = generate_occupancy_grid_polar(self.current_scan)[0]
            # self.get_logger().info(f"[SCAN] Stored lidar scan: {len(scan_msg.ranges)} points, range: [{scan_msg.range_min:.2f}, {scan_msg.range_max:.2f}], angles: [{scan_msg.angle_min:.2f}, {scan_msg.angle_max:.2f}]")
        except Exception as e:
            self.get_logger().error(f"[SCAN] Error processing scan: {e}", exc_info=True)
    
    def process_image(self, cv_image):
        """
        Process the image to match the original Panda3D implementation format.
        
        Args:
            cv_image: OpenCV image from Gazebo
            
        Returns:
            Processed image matching the original format (150x90, normalized)
        """
        # Convert to grayscale if needed
        if len(cv_image.shape) == 3:
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray_image = cv_image
        
        # Resize to target dimensions (150x90)
        resized_image = cv2.resize(gray_image, (self.VX, self.VY), interpolation=cv2.INTER_AREA)
        
        # Normalize to [0, 1] range
        normalized_image = resized_image.astype(np.float32) / 255.0
        # self.get_logger().info(f"Normalized image shape: {normalized_image.shape}")
        
        # Apply Gaussian blur to reduce noise (optional, matching original)
        # blurred_image = cv2.GaussianBlur(normalized_image, (3, 3), 0)
        
        return normalized_image
    
    def gen_neural_rate(self, image):
        """
        Generate neural rate using MATLAB engine.
        
        Args:
            image: Processed image array
            
        Returns:
            Neural rate output from MATLAB
        """
        try:
            S = np.zeros([100, 1])
            U = np.zeros([100, 1])
            
            # Convert numpy array to MATLAB format
            matlab_image = matlab.double(image.tolist())
            matlab_S = matlab.double(S.tolist())
            matlab_U = matlab.double(U.tolist())
            
            # Call MATLAB function
            S_out, U_out = self.eng.generate_V1_RSC_model_response(
                matlab_image, matlab_S, matlab_U, nargout=2
            )
            
            # Convert back to numpy
            neural_rate = np.array(S_out)
            
            return neural_rate
            
        except Exception as e:
            self.get_logger().error(f"Error in MATLAB neural rate generation: {e}")
            return np.zeros([100, 1])
    
    def find_cell(self, position):
        """Find which cell the robot is currently in"""
        # try:
        ls_flag = []
        for i in range(len(self.cell_ls)):
            ls_flag.append(self.cell_ls[i].check_in_polygon(np.reshape(position, (1, 2))))
        
        cell_indices = [i for i, x in enumerate(ls_flag) if x]
        # if cell_indices:
        return cell_indices[0]
        #     else:
        #         self.get_logger().warn("Robot not in any defined cell")
        #         return 0  # Default to first cell
                
        # except Exception as e:
        #     self.get_logger().error(f"Error finding cell: {e}")
        #     return 0
    
    def controller(self, neural_map, position, orientation):
        """
        Control system based on neural map analysis.
        
        Args:
            neural_map: Neural rate output
            position: Current robot position
            orientation: Current robot orientation
            
        Returns:
            Control input vector
        """
        try:
            # current_cell = self.find_cell(position)
            # current_cell = 48
            # Load control gains based on orientation
            K, Kb = self.control_gain_load.interpolate_contorlgains(self.current_cell, orientation)
            self.get_logger().info(f"cell_i: {self.current_cell}, orientation: {orientation}")
            # Calculate control input
        
            # if self.lidar_mode:
            #     measurement = self.encode_lidar_with_vae(self.current_scan['ranges'])
            #     u = K@measurement+Kb
            # else:

            if self.measurement_mode == 'neural_rate':
                u = K@neural_map+Kb
            elif self.measurement_mode == 'neural_lidar':
               
                measurement = []
                for i in range(len(self.RSC_data)):
                    measurement.append(np.sum(self.RSC_data[i]*self.current_grid_occ.T))

                measurement = np.array(measurement)
                measurement = measurement.reshape(-1, 1)
                # self.get_logger().info('measurement shape', measurement.shape)
                # self.get_logger().info('controller shape', K.shape)
                u = K[0]@measurement+Kb
           
            else:
                raise ValueError(f"Invalid measurement mode: {self.measurement_mode}")
            # if current_cell == 9:
            #     u[0] = 0
            
            # Normalize and scale
            speed = 100
            u_normalized = u / np.linalg.norm(u)
            u_scaled = u_normalized * speed
            # u_scaled = np.array([[0], [0]])
            self.get_logger().info(f"Control input %.3f, %.3f" % (u_scaled.flatten()[0], u_scaled.flatten()[1]))
            # print(f"Control input: {u_scaled.flatten()}")
            
            return u_scaled.reshape(2,1)
            
        except Exception as e:
            self.get_logger().error(f"Error in controller: {e}")
            return np.array([[0.0], [0.0]])




    def offest_unicycle_model(self, u):
        # Map to v, omega
        # epsilon is the offset of the unicycle model
        self.epsilon = 0.01
        # self.epsilon = 0.1
        J_inv = np.array([
            [np.cos(self.current_hd), np.sin(self.current_hd)],
            [-np.sin(self.current_hd)/self.epsilon, np.cos(self.current_hd)/self.epsilon]
        ])
        v_omega = np.dot(J_inv, u/900.0)
        v, omega = v_omega[0], v_omega[1]
        v = self.clamp(v, -10, 10)
        omega = self.clamp(omega, -30, 30)
        self.get_logger().info(f"v: {v}, omega: {omega}")
        return v, omega

    def publish_control(self, v, omega):
        'publish the control to the unicycle model'
        twist_msg = Twist()
        twist_msg.linear.x = float(v)
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = float(omega)
        self._control_pub.publish(twist_msg)


   
    def clamp(self, x: float, lo: float, hi: float) -> float:
        'clamp the value to the range [lo, hi]'
        return max(lo, min(hi, x))

    
    def process_callback(self):
        """Main processing callback"""
        if self.current_step == 0:
            # self.publish_robot_pose(self.current_position, self.current_hd)
            self.get_logger().info("initializing the robot...")
            self.current_step += 1

        elif self.current_step < self.num_steps:
            try:
              
                
               
                # neural_rate = self.gen_neural_rate(self.current_image)
                neural_rate = np.array([0,0])
                if self.current_step == 0:
                    self.get_logger().info("Starting neural analysis...")
                
                
                
                # self.get_logger().info(f"Position: {self.current_position}, Heading: {self.current_hd:.1f}°")
                self.current_u = self.controller(neural_rate, self.current_position, self.current_hd)
                self.current_v, self.current_omega = self.offest_unicycle_model(self.current_u)
    
                self.publish_control(self.current_v, self.current_omega)
                
              
                
            
                    
                # Save data periodically
                self.append_data()
                self.save_data_json()
                self.save_data()
                time.sleep(0.1)
                # if self.current_step % 10 == 0:
                #     self.save_data()
                
                # Save debug image
                # debug_filename = f'test/img/step_{self.current_step:03d}_pos_{self.current_position[0]:.2f}_{self.current_position[1]:.2f}_hd_{self.current_hd:.1f}.png'
                # plt.imsave(debug_filename, self.current_image, cmap='gray')
                # np.save(f'test/matx/step_{self.current_step:03d}_pos_{self.current_position[0]:.2f}_{self.current_position[1]:.2f}_hd_{self.current_hd:.1f}.npy', self.current_image)
                self.current_step += 1
                # self.save_final_data()
                
            except Exception as e:
                self.get_logger().error(f"Error in process callback: {e}")
        
        else:
            # Processing complete
            self.get_logger().info("Neural analysis completed!")
            self.save_final_data()
            self.cleanup()


    def save_data_json(self):
        """
        Save neural rate, heading, and state data as JSON
        save image and neural rate
        """
       
        timestamp = datetime.now().isoformat()
     
        heading_str = f"{self.current_hd_degree  :g}"
       
        data_entry = {
            "timestamp": timestamp,
            "step_number": int(self.current_step),
            "cell_id": int(self.current_cell),
            "heading": float(self.current_hd_degree ),
            "position": {
                "x": float(self.current_position[0]),
                "y": float(self.current_position[1])
            },
            "control": {
                "u_x": float(self.current_u[0]),
                "u_y": float(self.current_u[1]),
                "v": float(self.current_v),
                "omega": float(self.current_omega)
            },
            # "neural_rate": self.neural_rate.flatten().tolist()
        }
        
        # Add lidar scan data if available
        if self.current_scan is not None:
            data_entry["lidar_scan"] = self.current_scan
        else:
            self.get_logger().warn(f"No lidar scan data available for step {self.current_step}")
            data_entry["lidar_scan"] = None
        pos_x, pos_y = float(self.current_position[0]), float(self.current_position[1])
        json_path = os.path.join('test/data', f'step_{self.current_step:03d}_pos_{pos_x:.2f}_{pos_y:.2f}_hd_{self.current_hd_degree:g}.json')
        img_path = os.path.join('test/img', f'step_{self.current_step:03d}_pos_{pos_x:.2f}_{pos_y:.2f}_hd_{self.current_hd_degree:g}.png')
        # Create directories if they don't exist
        
        with open(json_path, 'w') as f:
            json.dump(data_entry, f, indent=2)
        plt.imsave(img_path, self.current_image, cmap='gray')


    def append_data(self):
        """Append data to the lists"""
        self.v_ls.append(self.current_v)
        self.omega_ls.append(self.current_omega)
        self.postion_ls.append(self.current_position.copy())
        self.hd_ls.append(self.current_hd)
        self.odom_ls.append(self.current_velocity.copy())
        self.u_ls.append(self.current_u.copy())

        
    def save_data(self):
        """Save intermediate data"""
        try:
            np.save('trj/postion_ls.npy', self.postion_ls)
            np.save('trj/hd_ls.npy', self.hd_ls)
            np.save('trj/u_ls.npy', self.u_ls)
            np.save('trj/image_ls.npy', self.image_ls)
            np.save('trj/ratemap_ls.npy', self.ratemap_ls)
            np.save('trj/odom_ls.npy', self.odom_ls)
            np.save('trj/v_ls.npy', np.asarray(self.v_ls, dtype=float))
            np.save('trj/omega_ls.npy', np.asarray(self.omega_ls, dtype=float))
            self.get_logger().debug("Intermediate data saved")
            
        except Exception as e:
            self.get_logger().error(f"Error saving intermediate data: {e}")
    
    def save_final_data(self):
        """Save final processed data"""
        try:
            # Convert lists to numpy arrays
        
            # Save final data
            np.save('trj/postion_ls_final.npy', self.postion_ls)
            np.save('trj/hd_ls_final.npy', self.hd_ls)
            np.save('trj/u_ls_final.npy', self.u_ls)
            np.save('trj/image_ls_final.npy', self.image_ls)
            np.save('trj/ratemap_ls_final.npy', self.ratemap_ls)
            
            self.get_logger().info("Final data saved successfully")
            
            # Create trajectory plot
            # self.create_trajectory_plot()
            
            # Save batch JSON data
            self.save_batch_json_data()
            
        except Exception as e:
            self.get_logger().error(f"Error saving final data: {e}")
    
  
    def cleanup(self):
        """Cleanup resources"""
        try:
            # Stop the timer to prevent further callbacks
            if hasattr(self, 'timer'):
                self.timer.cancel()
                self.get_logger().info("Timer stopped")
            
            # Quit MATLAB engine
            if hasattr(self, 'eng'):
                self.eng.quit()
                self.get_logger().info("MATLAB engine closed")
            
            # Destroy the node
            self.get_logger().info("Shutting down Gazebo Neural Analysis Node")
            self.destroy_node()
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description="Gazebo Neural Analysis Node")
    parser.add_argument(
        '--mode',
        choices=['controller', 'gen_data', 'vector_feild'],
        default='controller',
        help="Operating mode for the node"
    )
    parser.add_argument(
        '--cells',
        type=int,
        nargs='+',
        # default=list(range(0,48)),
        default=[18],
        help="Cell IDs to include when generating data"
    )
    parser.add_argument(
        '--headings',
        type=float,
        nargs='+',
        # default=list(range(0, 360, 10)),
        default=[90],
        help="Heading angles (degrees) to iterate when generating data"
    )
    parsed_args, remaining = parser.parse_known_args(args=args)

    rclpy.init(args=remaining)
    
    node = None
    try:
        node = GazeboNeuralAnalysisNode(
            mode=parsed_args.mode,
            cell_id_list=parsed_args.cells,
            heading_list=parsed_args.headings
        )
        
        # Use spin_once in a loop to check for shutdown flag
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if hasattr(node, 'should_shutdown') and node.should_shutdown:
                if rclpy.ok():
                    rclpy.shutdown()
                break
                
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Node interrupted by user")
        else:
            print("Node interrupted by user")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Node error: {e}")
        else:
            print(f"Node error before initialization: {e}", file=sys.stderr)
    finally:
        # Ensure cleanup is called if not already done
        if node is not None:
            if not (hasattr(node, 'should_shutdown') and node.should_shutdown):
                try:
                    node.cleanup()
                except:
                    pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
