#!/usr/bin/env python3
"""
Script to plot vector field from JSON data by:
1. Loading lidar data from JSON
2. Encoding lidar ranges with VAE
3. Computing control vectors using controller (similar to gazebo_neural_analysis.py)
4. Plotting the vector field
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import os
from pathlib import Path
import argparse
from gen_controller import vectorize_matrix
# Import dependencies
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. VAE functionality will be disabled.")

from vaemodel import load_vae_model
from cell_configs import cell_ls
from find_controller_orientation import control_gain_load
from visualize_lidar_scan import polar_to_cartesian



def generate_occupancy_grid_scan(lidar_scan, grid_size=10, front_angle_range=(-np.pi/3, np.pi/3) ):
    """
    Generate a 10x10 occupancy grid map based on lidar scan data.
    
    Args:
        json_file: Path to JSON file containing lidar scan data
        grid_size: Size of the occupancy grid (default: 10 for 10x10 grid)
        front_angle_range: Optional tuple (min_angle, max_angle) in radians to filter front points.
                          If None, uses all angles. Typical front range: (-np.pi/2, np.pi/2) for ±90°,
                          or (-np.pi/4, np.pi/4) for ±45° around front (0 radians).
    
    Returns:
        occupancy_grid: 2D numpy array of shape (grid_size, grid_size) with values:
            - 1.0: occupied cell (contains obstacles)
            - 0.0: free cell (along ray path, no obstacles)
            - 0.5: unknown cell (not explored)
        bounds: Dictionary with 'x_min', 'x_max', 'y_min', 'y_max' defining the grid bounds
    """
    # Load data
    # data = load_lidar_data(json_file)
    # lidar_scan = data['lidar_scan']
    
    # Extract lidar parameters
    ranges = np.array(lidar_scan['ranges'])
    angle_min = lidar_scan['angle_min']
    angle_max = lidar_scan['angle_max']
    angle_increment = lidar_scan['angle_increment']
    range_min = lidar_scan['range_min']
    range_max = lidar_scan['range_max']
    
    # Robot position (assumed at origin)
    robot_x = 0.0
    robot_y = 0.0
    
    # Convert to cartesian coordinates
    x, y, angles = polar_to_cartesian(ranges, angle_min, angle_max, angle_increment)
    
    # Filter out invalid ranges
    valid_mask = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    
    # Filter by front angle range if specified
    if front_angle_range is not None:
        angle_min_filter, angle_max_filter = front_angle_range
        front_mask = (angles >= angle_min_filter) & (angles <= angle_max_filter)
        valid_mask = valid_mask & front_mask
    
    x_valid = x[valid_mask]
    y_valid = y[valid_mask]
    ranges_valid = ranges[valid_mask]
    angles_valid = angles[valid_mask]
    
    # Determine grid bounds based on scan data
    # if len(x_valid) > 0:
    #     margin = 0.5
    #     x_min = min(np.min(x_valid) - margin, robot_x - margin)
    #     x_max = max(np.max(x_valid) + margin, robot_x + margin)
    #     y_min = min(np.min(y_valid) - margin, robot_y - margin)
    #     y_max = max(np.max(y_valid) + margin, robot_y + margin)
    # else:
        # Fallback bounds if no valid data
    x_min, x_max = -1.0, 1.0
    y_min, y_max = -1.0, 1.0

    # Initialize occupancy grid (0.5 = unknown)
    occupancy_grid = np.full((grid_size, grid_size), 0.0, dtype=np.float32)
    
    # Calculate cell size
    cell_width = (x_max - x_min) / grid_size
    cell_height = (y_max - y_min) / grid_size
    
    # Function to convert world coordinates to grid indices
    def world_to_grid(wx, wy):
        """Convert world coordinates to grid indices."""
        col = int((wx - x_min) / cell_width)
        row = int((wy - y_min) / cell_height)
        # Clamp to valid grid indices
        col = np.clip(col, 0, grid_size - 1)
        row = np.clip(row, 0, grid_size - 1)
        return row, col
    
    # Mark occupied cells (cells containing obstacle endpoints)
    for wx, wy in zip(x_valid, y_valid):
        row, col = world_to_grid(wx, wy)
        occupancy_grid[row, col] = 1.0  # Occupied
    
    # Mark free cells (cells along ray path from robot to obstacle)
    # Use Bresenham-like line algorithm to mark cells along the ray
    for i, (wx, wy, r) in enumerate(zip(x_valid, y_valid, ranges_valid)):
        if r > 0:
            # Get grid position of obstacle
            end_row, end_col = world_to_grid(wx, wy)
            
            # Get grid position of robot
            start_row, start_col = world_to_grid(robot_x, robot_y)
            
            # Mark all cells along the line from robot to obstacle as free
            # (except the obstacle cell itself which is already marked as occupied)
            num_steps = max(abs(end_row - start_row), abs(end_col - start_col)) + 1
            
            for step in range(num_steps):
                if num_steps > 1:
                    t = step / (num_steps - 1)
                else:
                    t = 0
                row = int(start_row + t * (end_row - start_row))
                col = int(start_col + t * (end_col - start_col))
                
                # Keep obstacle cells as occupied, mark others as free
                if occupancy_grid[row, col] != 1.0:
                    occupancy_grid[row, col] = 0.0  # Free
    
    # Store bounds for reference
    bounds = {
        'x_min': x_min,
        'x_max': x_max,
        'y_min': y_min,
        'y_max': y_max,
        'cell_width': cell_width,
        'cell_height': cell_height
    }
    
    return occupancy_grid, bounds


def find_cell(position, cell_ls):
    """Find which cell the position is in"""
    ls_flag = []
    for i in range(len(cell_ls)):
        ls_flag.append(cell_ls[i].check_in_polygon(np.reshape(position, (1, 2))))
    
    cell_indices = [i for i, x in enumerate(ls_flag) if x]
    if cell_indices:
        print(f"cell_id={cell_indices[0]}")
        return cell_indices[0]
    else:
        print(f"Warning: Position {position} not in any defined cell, using cell 0")
        # return 37


def encode_lidar_with_vae(VAE, lidar_ranges, vae_data_min, vae_data_max, vae_device):
    """
    Encode lidar scan ranges using the VAE model.
    
    Args:
        VAE: Loaded VAE model
        lidar_ranges: numpy array or list of lidar ranges (shape: [640] or [N, 640])
        vae_data_min: Minimum value for normalization
        vae_data_max: Maximum value for normalization
        vae_device: Device to run VAE on
    
    Returns:
        numpy array: Encoded latent representation (shape: [latent_dim] or [N, latent_dim])
    """
    if VAE is None:
        raise ValueError("VAE model not loaded.")
    
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
    data_min = float(vae_data_min)
    data_max = float(vae_data_max)
    
    lidar_normalized = (lidar_ranges - data_min) / (data_max - data_min + 1e-8)
    
    # Convert to torch tensor with float32 dtype and move to device
    lidar_tensor = torch.from_numpy(lidar_normalized).float().to(vae_device)
    
    # Encode to latent space
    with torch.no_grad():
        mu, logvar = VAE.encode(lidar_tensor)
        # Use mean of latent distribution
        z = mu
    
    # Convert back to numpy
    z_numpy = z.cpu().numpy()
    
    # Return single sample or batch
    if single_sample:
        return z_numpy[0]
    else:
        return z_numpy


def compute_control_vector_vae(VAE, lidar_ranges, position, heading, 
                          vae_data_min, vae_data_max, vae_device,
                          control_gain_loader, cell_ls,):
    """
    Compute control vector from lidar data using VAE and controller.
    
    Args:
        VAE: Loaded VAE model
        lidar_ranges: Lidar range data
        position: Robot position [x, y]
        heading: Robot heading in degrees
        vae_data_min: VAE normalization minimum
        vae_data_max: VAE normalization maximum
        vae_device: VAE device
        control_gain_loader: control_gain_load instance
        cell_ls: List of cell objects
    
    Returns:
        Control vector [u_x, u_y]
    """
    # Find current cell
    current_cell = find_cell(position, cell_ls)
    
    # Load control gains based on orientation
    K, Kb = control_gain_loader.interpolate_contorlgains(current_cell, heading)
    
    # Encode lidar with VAE
    measurement = encode_lidar_with_vae(VAE, lidar_ranges, 
                                       vae_data_min, vae_data_max, 
                                       vae_device)
    
    # Ensure measurement is column vector
    if measurement.ndim == 1:
        measurement = measurement.reshape(-1, 1)
    
    # Calculate control input: u = K @ measurement + Kb
    u = K @ measurement + Kb
    
    # Normalize and scale (similar to gazebo_neural_analysis.py)
    speed = 3.0
    u_norm = np.linalg.norm(u)
    if u_norm > 1e-8:
        u_normalized = u / u_norm
        u_scaled = u_normalized * speed
    else:
        u_scaled = u
    
    return u_scaled.flatten()


def compute_control_vector_neuralrate(lidar_scan, neural_rate, position, heading,
                             control_gain_loader, cell_ls, measurement_mode):
    """
    Compute control vector from lidar data using neural lidar and controller.
    
    Args:
        lidar_ranges: Lidar range data
        position: Robot position [x, y]
        heading: Robot heading in degrees
        control_gain_loader: control_gain_load instance
        cell_ls: List of cell objects
    """
    # Find current cell
    current_cell = find_cell(position, cell_ls)
    
    # Load control gains based on orientation
    K, Kb = control_gain_loader.interpolate_contorlgains(current_cell, heading)
    
    # Encode lidar with neural lidar
    if measurement_mode == 'neural_rate':
        measurement = neural_rate
    elif measurement_mode == 'neural_lidar':
        grid_occ,bounds = generate_occupancy_grid(lidar_scan)
        measurement = np.multiply(neural_rate, vectorize_matrix(grid_occ).reshape(100))
    else:
        raise ValueError(f"Invalid measurement mode: {measurement_mode}")
    u = K@measurement+Kb
    speed = 3.0
    u_norm = np.linalg.norm(u)
    if u_norm > 1e-8:
        u_normalized = u / u_norm
        u_scaled = u_normalized * speed
    else:
        u_scaled = u
    
    return u_scaled.flatten()


def plot_vector_field_from_lidar(json_path='trj/vector_field_data.json',
                                 vae_model_path='lidar_vae_model.pth',
                                 output_path='trj/vector_field_plot_from_lidar.png',
                                 show_cells=True,
                                 measurement_mode='vae',
                                 device='cpu'):
    """
    Plot vector field from lidar data by encoding with VAE and computing control vectors.
    
    Args:
        json_path: Path to JSON file containing lidar data
        vae_model_path: Path to VAE model file
        output_path: Path to save the output plot
        show_cells: Whether to plot cell boundaries
        device: Device to run VAE on ('cpu' or 'cuda')
    
    Returns:
        fig, ax: Matplotlib figure and axes objects
    """
    # Load JSON data
    json_path = Path(json_path)
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")
    
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    if len(data) == 0:
        raise ValueError("JSON file is empty")
    
    print(f"Loaded {len(data)} data points from {json_path}")
    
    # Load VAE model
    if not TORCH_AVAILABLE:
        raise ImportError("PyTorch is required for VAE encoding. Please install PyTorch.")
    
    vae_model_path = Path(vae_model_path)
    if not vae_model_path.exists():
        raise FileNotFoundError(f"VAE model file not found: {vae_model_path}")
    
    print(f"Loading VAE model from {vae_model_path}...")
    VAE, vae_data_min, vae_data_max = load_vae_model(model_path=str(vae_model_path), device=device)
    vae_device = next(VAE.parameters()).device
    print(f"VAE model loaded successfully. Device: {vae_device}")
    
    # Initialize control gain loader
    control_gain_loader = control_gain_load(measurement_mode=measurement_mode)
    
    # Extract data and compute control vectors
    positions = []
    u_vectors = []
    headings = []
    
    print("Computing control vectors from lidar data...")
    for i, entry in enumerate(data):
        pos = entry.get('position', [0, 0])
        heading = entry.get('heading', 0)
        lidar_ranges = entry.get('lidar_ranges', [])
        neural_rate = entry.get('neural_rate', [])
        if not lidar_ranges or len(lidar_ranges) == 0:
            print(f"Warning: No lidar data for entry {i}, skipping")
            continue
        
        try:
            if measurement_mode == 'vae':
            # Compute control vector
                u = compute_control_vector_vae(
                    VAE, lidar_ranges, pos, heading, 
                    vae_data_min, vae_data_max, vae_device,
                    control_gain_loader, cell_ls
                )
            else:
                u = compute_control_vector_neuralrate(
                    lidar_scan, neural_rate, pos, heading,
                    control_gain_loader, cell_ls, measurement_mode
                )
           
            positions.append(pos)
            u_vectors.append(u)
            headings.append(heading)
            
            if (i + 1) % 10 == 0:
                print(f"Processed {i + 1}/{len(data)} entries...")
                
        except Exception as e:
            print(f"Error processing entry {i}: {e}")
            continue
    
    if len(positions) == 0:
        raise ValueError("No valid control vectors computed")
    
    # Convert to numpy arrays
    positions = np.array(positions)
    u_vectors = np.array(u_vectors)
    headings = np.array(headings)
    
    print(f"\nComputed {len(positions)} control vectors")
    print(f"Position range: x=[{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}], "
          f"y=[{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
    print(f"Control vector range: u_x=[{u_vectors[:, 0].min():.3f}, {u_vectors[:, 0].max():.3f}], "
          f"u_y=[{u_vectors[:, 1].min():.3f}, {u_vectors[:, 1].max():.3f}]")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot cell boundaries if requested
    if show_cells:
        try:
            # Plot cell boundaries
            for cell in cell_ls:
                vrt = np.array(cell.vrt)
                for i in range(len(vrt) - 1):
                    ax.plot([vrt[i, 0], vrt[i+1, 0]], 
                           [vrt[i, 1], vrt[i+1, 1]], 
                           color='gray', linewidth=1, alpha=0.7)
                ax.plot([vrt[0, 0], vrt[-1, 0]], 
                       [vrt[0, 1], vrt[-1, 1]], 
                       color='gray', linewidth=1, alpha=0.7)
                
                # Plot barriers in red
                for barrier in cell.bar:
                    ax.plot([barrier[0][0], barrier[1][0]], 
                           [barrier[0][1], barrier[1][1]], 
                           color='red', linewidth=2, alpha=0.8)
                
                # Plot exit in green
                exit_vrt = cell.exit_vrt
                ax.plot([exit_vrt[0][0], exit_vrt[1][0]], 
                       [exit_vrt[0][1], exit_vrt[1][1]], 
                       color='green', linewidth=2, alpha=0.8)
            
            print("Cell boundaries plotted")
        except Exception as e:
            print(f"Warning: Could not plot cell boundaries: {e}")
    
    # Calculate vector magnitudes for info
    magnitudes = np.sqrt(u_vectors[:, 0]**2 + u_vectors[:, 1]**2)
    max_magnitude = np.max(magnitudes)
    min_magnitude = np.min(magnitudes[magnitudes > 0]) if np.any(magnitudes > 0) else 0
    print(f"Vector magnitudes: min={min_magnitude:.4f}, max={max_magnitude:.4f}")
    
    # Plot vector field
    quiver = ax.quiver(positions[:, 0], positions[:, 1], 
                       u_vectors[:, 0], u_vectors[:, 1],
                       angles='xy', scale_units='xy', 
                       scale=None,  # Auto-scale
                       width=0.003,
                       alpha=0.7,
                       color='blue',
                       headwidth=3,
                       headlength=4)
    
    # Add labels and formatting
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Vector Field from Lidar Data (VAE Encoded)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Save the plot
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nVector field plot saved to: {output_path}")
    
    return fig, ax


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot vector field from lidar data using VAE')
    parser.add_argument('--json', type=str, default='trj/vector_field_data.json',
                        help='Path to JSON file containing lidar data')
    parser.add_argument('--vae-model', type=str, default='lidar_vae_model.pth',
                        help='Path to VAE model file')
    parser.add_argument('--output', type=str, default='trj/vector_field_plot_from_lidar.png',
                        help='Path to save the output plot')
    parser.add_argument('--no-cells', action='store_true', default=False,
                        help='Do not plot cell boundaries')
    parser.add_argument('--device', type=str, default='cpu',
                        choices=['cpu', 'cuda'],
                        help='Device to run VAE on')
    
    args = parser.parse_args()
    
    try:
        fig, ax = plot_vector_field_from_lidar(
            json_path=args.json,
            vae_model_path=args.vae_model,
            output_path=args.output,
            # show_cells=not args.no_cells,
            show_cells=True,
            device=args.device,
            measurement_mode='vae'
        )
        plt.show()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

