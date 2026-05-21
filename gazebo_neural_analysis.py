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
# Import torch for VAE model loading
try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. VAE functionality will be disabled.")

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



        self.current_position = np.array([0.25, 1.0])  # Initial position
        self.current_hd = -90  #Initial heading direction

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
            '/scan', 
            self.scan_callback, 
            scan_qos
        )
        self.get_logger().info("Subscribed to /scan topic for lidar data with sensor QoS profile")
        
        # Create synchronizer for image and pose data
        self.ts = ApproximateTimeSynchronizer([self.image_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        # Create publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.publisher_pose = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for processing
        self.timer_period = 0.1  # 10 Hz


    
        if self.measurement_mode == 'vae':
            # Load VAE model for lidar scan processing
            # Try multiple possible paths to find the model file
            # This is important because ROS2 nodes may run from different working directories
            # possible_paths = [
            #     'lidar_vae_model.pth',  # Relative to current working directory
            #     os.path.join(os.path.dirname(__file__), 'lidar_vae_model.pth'),  # Relative to script location
            #     os.path.expanduser('~/NerualRateMaps/lidar_vae_model.pth'),  # Absolute home path
            #     '/home/mehdi/NerualRateMaps/lidar_vae_model.pth'  # Hardcoded absolute path as fallback
            # ]
            
            # vae_model_path = None
            # for path in possible_paths:
            #     abs_path = os.path.abspath(path)
            #     if os.path.exists(abs_path):
            #         file_size = os.path.getsize(abs_path)
            #         if file_size > 1000:  # Valid file size
            #             vae_model_path = abs_path
            #             self.get_logger().info(f"Found VAE model at: {vae_model_path}")
            #             break
            
        
            self.VAE, self.vae_data_min, self.vae_data_max = load_vae_model(
                device='cpu'  # Use CPU by default, change to 'cuda' if GPU available
            )
            self.vae_device = 'cpu'
                    
    
        self.mode = mode
        if mode == 'controller':
            self.timer = self.create_timer(self.timer_period, self.process_callback)
        elif mode == 'gen_data' or mode == 'vector_feild':
            self._all_points(self.cell_id_list, self.heading_list)
            self.timer = self.create_timer(self.timer_period, self.gen_data_callback)
            if mode == 'vector_feild':
                self.all_data = []
        else:
            self.get_logger().error(f"Invalid mode: {mode}")
            sys.exit(1)        




     



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
        self.create_timer(1.0, self.check_scan_topic)
    
    def check_scan_topic(self):
        """Check if scan topic is available and logging"""
        if self.scan_check_done:
            return
        self.scan_check_done = True
        
        try:
            # Get topic info
            topics = self.get_topic_names_and_types()
            scan_topics = [name for name, types in topics if 'scan' in name.lower()]
            
            if scan_topics:
                self.get_logger().info(f"Found scan-related topics: {scan_topics}")
            else:
                self.get_logger().warn(f"WARNING: No scan topics found! Available topics: {[name for name, _ in topics[:10]]}")
            
            # Check if /scan specifically exists
            topic_names = [name for name, _ in topics]
            if '/scan' in topic_names:
                self.get_logger().info("✓ /scan topic is available")
                # Try to get topic info
                try:
                    publisher_count = len(self.get_publishers_info_by_topic('/scan'))
                    subscriber_count = len(self.get_subscriptions_info_by_topic('/scan'))
                    self.get_logger().info(f"  Publishers: {publisher_count}, Subscribers: {subscriber_count}")
                    
                    # Verify our subscription
                    sub_info = self.get_subscriptions_info_by_topic('/scan')
                    if sub_info:
                        self.get_logger().info(f"  Our subscription: {sub_info[0].topic_name}, QoS: {sub_info[0].qos_profile}")
                    else:
                        self.get_logger().warn("  WARNING: Could not find our subscription to /scan!")
                except Exception as e:
                    self.get_logger().error(f"  Error getting topic info: {e}")
            else:
                self.get_logger().error("✗ /scan topic NOT found! Check your Gazebo model configuration.")
                
        except Exception as e:
            self.get_logger().error(f"Error checking scan topic: {e}")
    
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
            current_cell = self.find_cell(position)
            # current_cell = 48
            # Load control gains based on orientation
            K, Kb = self.control_gain_load.interpolate_contorlgains(current_cell, orientation)
            self.get_logger().info(f"cell_i: {current_cell}, orientation: {orientation}")
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
            elif self.measurement_mode == 'vae':
                if self.current_scan is None:
                    self.get_logger().warn("No lidar scan data available yet, using zero measurement")
                    # Get latent dim from VAE model if available, otherwise default to 32
                    latent_dim = getattr(self.VAE, 'latent_dim', 32) if self.VAE else 32
                    measurement = np.zeros((latent_dim, 1))  # Match expected shape for matrix multiplication
                else:
                    measurement = self.encode_lidar_with_vae(self.current_scan['ranges'])
                    # Ensure measurement is column vector
                    if len(measurement.shape) == 1:
                        measurement = measurement.reshape(-1, 1)
                u = K@measurement+Kb
            else:
                raise ValueError(f"Invalid measurement mode: {self.measurement_mode}")
            # if current_cell == 9:
            #     u[0] = 0
            
            # Normalize and scale
            speed = 2.5
            u_normalized = u / np.linalg.norm(u)
            u_scaled = u_normalized * speed
            # u_scaled = np.array([[0], [0]])
            # self.get_logger().info(f"Control input: {u_scaled.flatten()}")
            # print(f"Control input: {u_scaled.flatten()}")
            
            return u_scaled.reshape(2,1)
            
        except Exception as e:
            self.get_logger().error(f"Error in controller: {e}")
            return np.array([[0.0], [0.0]])



    def encode_lidar_with_vae(self, lidar_ranges):
        """
        Encode lidar scan ranges using the VAE model.
        
        Args:
            lidar_ranges: numpy array of lidar ranges (shape: [640] or [N, 640])
        
        Returns:
            numpy array: Encoded latent representation (shape: [latent_dim] or [N, latent_dim])
        """
        if self.VAE is None:
            raise ValueError("VAE model not loaded. Set lidar_mode=True and ensure model file exists.")
        
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch is required for VAE encoding")
        
        # Convert to numpy array if needed
        if not isinstance(lidar_ranges, np.ndarray):
            lidar_ranges = np.array(lidar_ranges)
        
        # Handle single sample vs batch
        single_sample = len(lidar_ranges.shape) == 1
        if single_sample:
            lidar_ranges = lidar_ranges.reshape(1, -1)
        
        # Convert to numpy array and ensure float32
        if isinstance(lidar_ranges, torch.Tensor):
            lidar_ranges = lidar_ranges.cpu().numpy()
        lidar_ranges = np.array(lidar_ranges, dtype=np.float32)
        
        # Normalize data (same normalization as training)
        # Normalization parameters should be Python scalars (converted in load_vae_model)
        data_min = float(self.vae_data_min)
        data_max = float(self.vae_data_max)
        
        lidar_normalized = (lidar_ranges - data_min) / (data_max - data_min + 1e-8)
        
        # Convert to torch tensor with float32 dtype and move to device
        
        lidar_tensor = torch.from_numpy(lidar_normalized).float().to(self.vae_device)
        
        # Encode to latent space
        with torch.no_grad():
            mu, logvar = self.VAE.encode(lidar_tensor)
            # Use mean of latent distribution (or use reparameterize for sampling)
            z = mu  # Or use: z = self.VAE.reparameterize(mu, logvar) for sampling
        
        # Convert back to numpy
        z_numpy = z.cpu().numpy()
        
        # Return single sample or batch
        if single_sample:
            return z_numpy[0]
        else:
            return z_numpy
    
    def update_state(self, neural_map, position, orientation, update_hd= True):
        """
        Update robot state based on neural map and control input.
        
        Args:
            neural_map: Neural rate output
            position: Current position
            orientation: Current orientation
            update_hd: Whether to update heading direction
            
        Returns:
            New position and heading direction
        """
        try:
            # Get control input
            u = self.controller(neural_map, position, orientation)
            self.u_ls.append(u)
            
            # Update heading direction
            if update_hd:
                new_hd = (np.arctan2(u[1], u[0]) * 180 / np.pi) % 360
                
                # Smooth heading update
                if np.abs(new_hd + 360 - orientation) < np.abs(new_hd - orientation):
                    new_hd = new_hd + 360
                
                eta = 0.2
                new_hd = ((1 - eta) * orientation + eta * float(new_hd))
                new_hd = np.sign(new_hd - orientation) * min(np.abs(new_hd - orientation), 18) + orientation
                new_hd = new_hd % 360
            else:
                new_hd = orientation
            
            # Update position
            B_dis = np.eye(2) * self.dt
            new_position = position + (B_dis @ u).flatten()
            self.postion_ls.append(new_position.copy())
            self.hd_ls.append(new_hd)
        
           
            
            return new_position, new_hd
            
        except Exception as e:
            self.get_logger().error(f"Error updating state: {e}")
            return position, orientation
    
    def publish_robot_pose(self, position, orientation):
        """Publish robot pose for visualization"""
        try:
            msg = JointTrajectory()
            msg.header = Header()
            msg.header.frame_id = "footprint_link"
            msg.joint_names = ["x_pose", "y_pose", "rot_joint"]

            point = JointTrajectoryPoint()
            # Convert to Python float to ensure ROS2 compatibility
            x_pos = float(position[0])
            y_pos = float(position[1])
            orientation_rad = float(orientation)*np.pi/180
            point.positions = [x_pos, y_pos, orientation_rad]
            point.time_from_start.sec = 1  # Change as needed

            msg.points = [point]    
            self.publisher_pose.publish(msg)
            
            self.get_logger().debug('Publishing joint trajectory')
            # time.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing pose: {e}")

   
    
    def save_json_data(self, neural_rate, position, heading, step_number, timestamp=None):
        """
        Save neural rate, heading, and state data as JSON
        
        Args:
            neural_rate: Neural rate output from MATLAB
            position: Current robot position [x, y]
            heading: Current heading direction (degrees)
            # control_input: Control input vector [u_x, u_y]
            step_number: Current processing step
            timestamp: Optional timestamp (defaults to current time)
        """
        try:
            if timestamp is None:
                timestamp = datetime.now().isoformat()
            
            # Create data dictionary
            data_entry = {
                "timestamp": timestamp,
                "step_number": int(step_number),
                "position": {
                    "x": float(position[0]),
                    "y": float(position[1])
                },
                "heading": float(heading),
                "neural_rate": neural_rate.flatten().tolist(),
                # "control_input": {
                #     "u_x": float(control_input[0]),
                #     "u_y": float(control_input[1])
                # },
                "neural_rate_stats": {
                    "mean": float(np.mean(neural_rate)),
                    "std": float(np.std(neural_rate)),
                    "min": float(np.min(neural_rate)),
                    "max": float(np.max(neural_rate))
                }
            }
            
            # Add to list for batch saving
            self.json_data_list.append(data_entry)
            
            # Save individual JSON file for this step
            individual_filename = os.path.join(self.json_data_dir, f'step_{step_number:03d}.json')
            # sa
            with open(individual_filename, 'w') as f:
                json.dump(data_entry, f, indent=2)
            
            self.get_logger().debug(f"JSON data saved for step {step_number}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving JSON data: {e}")


    def save_gen_data(self, neural_rate, position, heading, step_number, timestamp=None, cell_id=None):
        """
        Save neural rate, heading, and state data as JSON
        save image and neural rate
        """
        if timestamp is None:
            timestamp = datetime.now().isoformat()
            
        if cell_id is None:
            cell_id = self.find_cell(position)

        heading_str = f"{heading:g}"
        neural_dir = os.path.join('cells_kernels', f'c{int(cell_id)}', f'deg{heading_str}')
        image_dir = os.path.join('cells_kernels_images', f'c{int(cell_id)}', f'deg{heading_str}')
        neural_rate_path = os.path.join(neural_dir, f'nr_{position[0]:.2f}_y{position[1]:.2f}_HD{heading_str}.json')
        neural_rate_path_npy = os.path.join(neural_dir, f'nr_{position[0]:.2f}_y{position[1]:.2f}_HD{heading_str}.npy')
        image_path = os.path.join(image_dir, f'img_{position[0]:.2f}_y{position[1]:.2f}_HD{heading_str}.png')
          
        data_entry = {
            "timestamp": timestamp,
            "step_number": int(step_number),
            "cell_id": int(cell_id),
            "heading": float(heading),
            "position": {
                "x": float(position[0]),
                "y": float(position[1])
            },
            "neural_rate": neural_rate.flatten().tolist()
        }
        
        # Add lidar scan data if available
        if self.current_scan is not None:
            data_entry["lidar_scan"] = self.current_scan
        else:
            self.get_logger().warn(f"No lidar scan data available for step {step_number}")
            data_entry["lidar_scan"] = None
        
        # Create directories if they don't exist
        os.makedirs(neural_dir, exist_ok=True)
        os.makedirs(image_dir, exist_ok=True)
        np.save(neural_rate_path_npy, neural_rate)
        with open(neural_rate_path, 'w') as f:
            json.dump(data_entry, f, indent=2)
        plt.imsave(image_path, self.current_image, cmap='gray')

    def save_batch_json_data(self):
        """Save all collected JSON data as a single file"""
        batch_filename = os.path.join(self.json_data_dir, 'all_steps_data.json')
        with open(batch_filename, 'w') as f:
            json.dump(self.json_data_list, f, indent=2)
        
        self.get_logger().info(f"Batch JSON data saved: {len(self.json_data_list)} entries")
        
    
    def process_callback(self):
        """Main processing callback"""
        if self.current_step == 0:
            self.publish_robot_pose(self.current_position, self.current_hd)
            self.get_logger().info("initializing the robot...")
            self.current_step += 1

        elif self.current_step < self.num_steps:
            try:
                # Check if we have a current image
                # if not hasattr(self, 'current_image'):
                #     self.get_logger().warn("No image available for processing")
                #     return
                
                # Generate neural rate from current image
                # neural_rate = self.gen_neural_rate(self.current_image)
                neural_rate = np.array([0,0])
                if self.current_step == 0:
                    self.get_logger().info("Starting neural analysis...")
                
                # self.get_logger().info(f"Neural Rate: {neural_rate.flatten()[:5]}...")  # Log first 5 values
                
                # Update robot state
                self.current_position, self.current_hd = self.update_state(
                    neural_rate, self.current_position, self.current_hd
                )
                
                self.get_logger().info(f"Position: {self.current_position}, Heading: {self.current_hd:.1f}°")
                
                # Get control input for JSON logging (from the last update_state call)
                current_control_input = self.u_ls[-1] if self.u_ls else np.array([[0.0], [0.0]])
                
                # Save JSON data for this step
                if self.mode == 'gen_data':
                    self.save_json_data(
                        neural_rate=neural_rate,
                        position=self.current_position,
                        heading=self.current_hd,
                        # control_input=current_control_input,
                        step_number=self.current_step
                    )
                    
                    # Store data
                    self.postion_ls.append(self.current_position.copy())
                    self.hd_ls.append(self.current_hd)
                    self.ratemap_ls.append(neural_rate.copy())
                    self.image_ls.append(self.current_image.copy())
                    
                # Save data periodically
                if self.current_step % 10 == 0:
                    self.save_data()
                
                # Publish robot pose
                self.publish_robot_pose(self.current_position, self.current_hd)
                
                # Save debug image
                debug_filename = f'test/img/step_{self.current_step:03d}_pos_{self.current_position[0]:.2f}_{self.current_position[1]:.2f}_hd_{self.current_hd:.1f}.png'
                plt.imsave(debug_filename, self.current_image, cmap='gray')
                np.save(f'test/matx/step_{self.current_step:03d}_pos_{self.current_position[0]:.2f}_{self.current_position[1]:.2f}_hd_{self.current_hd:.1f}.npy', self.current_image)
                self.current_step += 1
                self.save_final_data()
                
            except Exception as e:
                self.get_logger().error(f"Error in process callback: {e}")
        
        else:
            # Processing complete
            self.get_logger().info("Neural analysis completed!")
            self.save_final_data()
            self.cleanup()

    def _all_points(self, cell_id_ls, heading_ls):
        self.grid_targets = []
        for cell_id in cell_id_ls:
            if self.mode == 'vector_feild':
                num_points = 6
            else:
                num_points = 6
            X, Y = gen_grid_points(cell_id, num_points)
            for heading in heading_ls:
                for x, y in zip(X, Y):
                    self.grid_targets.append(
                        {
                            "cell_id": int(cell_id),
                            "position": np.array([float(x), float(y)]),
                            "heading": float(heading)
                        }
                    )
        self.num_steps = len(self.grid_targets)
        self.current_step = 0
        if self.grid_targets:
            self.current_position = self.grid_targets[0]["position"].copy()
            self.current_hd = self.grid_targets[0]["heading"]
            self.current_cell_id = self.grid_targets[0]["cell_id"]
        else:
            self.current_position = np.zeros(2)
            self.current_hd = heading_ls[0] if heading_ls else 0.0
            self.current_cell_id = cell_id_ls[0] if cell_id_ls else 0
      


    def vector_field_collected_data(self, cell_id, heading):
        ux = []
        uy = []
        X = []
        Y = []
        directory_data = os.path.join('cells_kernels', f'c{int(cell_id)}', f'deg{heading}')
        files = os.listdir(directory_data)
        for file in files:
            if file.endswith('.json'):
                with open(os.path.join(directory_data, file), 'r') as f:
                    data = json.load(f)
                    neural_rate = np.array(data["neural_rate"]).reshape(-1,1)
                    position = np.array([[data["position"]["x"]], [data["position"]["y"]]])
                    heading = float(data["heading"])
                u = self.controller(neural_rate, position, heading)
                ux.append(u[0])
                uy.append(u[1])
                X.append(position[0][0])
                Y.append(position[1][0])
        ##plot vector field
        fig, ax = plt.subplots()
        ax.quiver(X, Y, ux, uy)
        # plt.show()
        plt.savefig(f'trj/vector_field_plot_c{cell_id}_deg{heading}.png', dpi=300)
        plt.close()
                
    def vector_field_plot(self):
        '''plot vector field'''
        if not self.all_data:
            self.get_logger().warn("No data available for vector field plot")
            return
        with open('trj/vector_field_data.json', 'w') as f:
            json.dump(self.all_data, f)
        # Extract positions and control inputs from all_data
        positions = np.array([entry["position"] for entry in self.all_data])
        u_vectors = np.array([entry["u"] for entry in self.all_data])
        
        fig, ax = plt.subplots()
        ax.quiver(positions[:, 0], positions[:, 1], u_vectors[:, 0], u_vectors[:, 1])
        # plt.show()
        plt.savefig(f'trj/vector_field_plot_c{self.current_cell_id}_deg{self.current_hd}.png', dpi=300)
        plt.close()
        #save data to json
        


    def gen_data_callback(self):
        """Generate data callback - ensures pose is updated before capturing image"""
        if not hasattr(self, 'grid_targets') or not self.grid_targets:
            self.get_logger().error("No grid targets defined for gen_data mode")
            self.should_shutdown = True
            self.cleanup()
            return

        if self.current_step >= self.num_steps:
            # Processing complete
            self.get_logger().info("Neural analysis completed!")
            if self.mode == 'vector_feild':
                self.vector_field_plot()
            self.should_shutdown = True
            self.cleanup()
            return
        
        try:
            import time
            current_target = self.grid_targets[self.current_step]
            position = current_target["position"]
            heading = current_target["heading"]
            cell_id = current_target["cell_id"]
            
            # Step 1: Publish pose if not done yet for this step
            if not self.pose_published:
                self.current_position = position.copy()
                self.current_hd = heading
                self.current_cell_id = cell_id
                self.publish_robot_pose(self.current_position, self.current_hd)
                self.pose_published = True
                self.pose_publish_time = time.time()
                self.get_logger().info(
                    f"Step {self.current_step + 1}/{self.num_steps}: Published pose at "
                    f"({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) "
                    f"with heading {self.current_hd:.1f}°, waiting for image update..."
                )
                return  # Exit and wait for next callback
            
            # Step 2: Wait for image to be updated after pose was published
            # Give some time for Gazebo to update and for image to arrive (at least 2-3 callbacks)
            if not hasattr(self, 'pose_publish_time'):
                self.pose_publish_time = time.time()
            
            time_since_pose = time.time() - self.pose_publish_time
            
            if time_since_pose < 0.2:  # Wait at least 200ms for image to update
                self.get_logger().debug(f"Waiting for image update... ({time_since_pose:.2f}s elapsed)")
                return
            
            # Step 3: Check if we have a current image
            # if not hasattr(self, 'current_image'):
            #     self.get_logger().warn("No image available for processing")
            #     return
            
            # Step 4: Process the image
            self.get_logger().info(
                f"Processing image for step {self.current_step + 1} "
                f"(cell {cell_id}, heading {heading:.1f}°)"
            )
            
            # Generate neural rate from current image
            # neural_rate = self.gen_neural_rate(self.current_image)
            neural_rate = np.array([0,0])
            if self.mode == 'vector_feild':
                u = self.controller(neural_rate, self.current_position, self.current_hd)
                self.all_data.append({
                    "neural_rate": neural_rate.flatten().tolist().copy(),
                    "u": u.flatten().tolist().copy(),
                    "position": self.current_position.tolist().copy(),
                    "heading": self.current_hd,
                    "lidar_data": self.current_scan
                })
                
                
            
            # self.get_logger().info(f"Neural Rate: {neural_rate.flatten()[:5]}...")  # Log first 5 values
            
            # Update robot state
            self.current_position = position.copy()
            self.current_hd = heading
            
            self.get_logger().info(
                f"Position: {self.current_position}, Heading: {self.current_hd:.1f}°"
            )
            
            # Save the data
            if self.mode == 'gen_data':
                self.save_gen_data(
                    neural_rate,
                    self.current_position,
                    self.current_hd,
                    self.current_step,
                    cell_id=cell_id
                )
                
            # Move to next step and reset state
            self.current_step += 1
            self.pose_published = False  # Reset for next step
            
            self.get_logger().info(f"Step {self.current_step-1} completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error in gen_data callback: {e}")
            # Reset state to try again
            self.pose_published = False
    
    def save_data(self):
        """Save intermediate data"""
        try:
            np.save('trj/postion_ls.npy', self.postion_ls)
            np.save('trj/hd_ls.npy', self.hd_ls)
            np.save('trj/u_ls.npy', self.u_ls)
            np.save('trj/image_ls.npy', self.image_ls)
            np.save('trj/ratemap_ls.npy', self.ratemap_ls)
            
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
    
    def create_trajectory_plot(self):
        """Create trajectory visualization plot"""
        try:
            fig, ax = plt.subplots(figsize=(12, 8))
            
            # Plot trajectory
            ax.plot(self.postion_ls[:, 0], self.postion_ls[:, 1], 'b-', linewidth=2, label='Trajectory')
            ax.scatter(self.postion_ls[0, 0], self.postion_ls[0, 1], color='green', s=100, label='Start')
            ax.scatter(self.postion_ls[-1, 0], self.postion_ls[-1, 1], color='red', s=100, label='End')
            
            # Plot control vectors
            for i in range(0, len(self.u_ls), 5):  # Every 5th vector
                ax.quiver(self.postion_ls[i, 0], self.postion_ls[i, 1], 
                         self.u_ls[i, 0], self.u_ls[i, 1], 
                         scale=10, alpha=0.7, color='orange')
            
            ax.set_xlabel('X Position')
            ax.set_ylabel('Y Position')
            ax.set_title('Robot Trajectory with Control Vectors')
            ax.legend()
            ax.grid(True)
            ax.axis('equal')
            
            plt.tight_layout()
            plt.savefig('trj/trajectory_plot.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info("Trajectory plot saved")
            
        except Exception as e:
            self.get_logger().error(f"Error creating trajectory plot: {e}")
    
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
