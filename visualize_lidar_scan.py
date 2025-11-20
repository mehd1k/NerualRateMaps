import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch


def load_lidar_data(json_file):
    """Load lidar scan data from JSON file."""
    with open(json_file, 'r') as f:
        data = json.load(f)
    return data


def polar_to_cartesian(ranges, angle_min, angle_max, angle_increment):
    """Convert polar coordinates (range, angle) to cartesian (x, y)."""
    # Generate angles for each range measurement
    num_points = len(ranges)
    if angle_increment > 0:
        angles = np.arange(angle_min, angle_min + num_points * angle_increment, angle_increment)
        angles = angles[:num_points]  # Ensure same length as ranges
    else:
        # Fallback: evenly distribute angles between min and max
        angles = np.linspace(angle_min, angle_max, num_points)
    
    # Convert to cartesian coordinates
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    return x, y, angles


def visualize_lidar_scan(json_file, show_robot=True, show_rays=True, figsize=(10, 10)):
    """
    Visualize lidar scan from JSON file.
    
    Args:
        json_file: Path to JSON file containing lidar scan data
        show_robot: Whether to show robot position and heading
        show_rays: Whether to draw lines from robot to scan points
        figsize: Figure size (width, height)
    """
    # Load data
    data = load_lidar_data(json_file)
    lidar_scan = data['lidar_scan']
    
    # Extract lidar parameters
    ranges = np.array(lidar_scan['ranges'])
    angle_min = lidar_scan['angle_min']
    angle_max = lidar_scan['angle_max']
    angle_increment = lidar_scan['angle_increment']
    range_min = lidar_scan['range_min']
    range_max = lidar_scan['range_max']
    
    # Extract robot position and heading if available
    # robot_x = data.get('position', {}).get('x', 0.0)
    # robot_y = data.get('position', {}).get('y', 0.0)
    robot_heading = data.get('heading', 0.0)
    robot_x = 0
    robot_y = 0

    
    # Convert to cartesian coordinates
    x, y, angles = polar_to_cartesian(ranges, angle_min, angle_max, angle_increment)
    
    # Filter out invalid ranges (NaN, inf, or out of range)
    valid_mask = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    x_valid = x[valid_mask]
    y_valid = y[valid_mask]
    ranges_valid = ranges[valid_mask]
    
    # Create figure
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot scan points
    # Color points by distance for better visualization
    scatter = ax.scatter(x_valid, y_valid, c=ranges_valid, cmap='viridis', 
                        s=20, alpha=0.6, edgecolors='black', linewidths=0.5)
    plt.colorbar(scatter, ax=ax, label='Range (m)')
    
    # Draw rays from robot to scan points (optional)
    if show_rays and show_robot:
        # Sample rays to avoid cluttering (show every nth ray)
        step = max(1, len(x_valid) // 100)
        for i in range(0, len(x_valid), step):
            if valid_mask[i]:
                ax.plot([0, x[i]], [0, y[i]], 'gray', alpha=0.2, linewidth=0.5)
    
    # Show robot position and heading
    if show_robot:
        # Robot position
        ax.plot(robot_x, robot_y, 'ro', markersize=10, label='Robot Position', zorder=10)
        
        # Robot heading arrow
        arrow_length = 0.3
        arrow_dx = arrow_length * np.cos(robot_heading)
        arrow_dy = arrow_length * np.sin(robot_heading)
        arrow = FancyArrowPatch((robot_x, robot_y), 
                               (robot_x + arrow_dx, robot_y + arrow_dy),
                               arrowstyle='->', mutation_scale=20, 
                               color='red', linewidth=2, label='Heading', zorder=10)
        ax.add_patch(arrow)
        
        # Draw a circle around robot
        circle = Circle((robot_x, robot_y), 0.1, fill=False, 
                       color='red', linestyle='--', linewidth=1, alpha=0.5)
        ax.add_patch(circle)
    
    # Set equal aspect ratio
    ax.set_aspect('equal')
    
    # Labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Lidar Scan Visualization')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    
    # Set reasonable axis limits based on scan data
    if len(x_valid) > 0:
        margin = 0.5
        x_min, x_max = np.min(x_valid) - margin, np.max(x_valid) + margin
        y_min, y_max = np.min(y_valid) - margin, np.max(y_valid) + margin
        
        # Center around robot if shown
        if show_robot:
            x_range = max(abs(x_min - robot_x), abs(x_max - robot_x)) + margin
            y_range = max(abs(y_min - robot_y), abs(y_max - robot_y)) + margin
            ax.set_xlim(robot_x - x_range, robot_x + x_range)
            ax.set_ylim(robot_y - y_range, robot_y + y_range)
        else:
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
    
    plt.tight_layout()
    return fig, ax


def visualize_lidar_scan_polar(json_file, figsize=(12, 5)):
    """
    Visualize lidar scan in polar coordinates.
    
    Args:
        json_file: Path to JSON file containing lidar scan data
        figsize: Figure size (width, height)
    """
    # Load data
    data = load_lidar_data(json_file)
    lidar_scan = data['lidar_scan']
    
    # Extract lidar parameters
    ranges = np.array(lidar_scan['ranges'])
    angle_min = lidar_scan['angle_min']
    angle_max = lidar_scan['angle_max']
    angle_increment = lidar_scan['angle_increment']
    
    # Generate angles
    num_points = len(ranges)
    if angle_increment > 0:
        angles = np.arange(angle_min, angle_min + num_points * angle_increment, angle_increment)
        angles = angles[:num_points]
    else:
        angles = np.linspace(angle_min, angle_max, num_points)
    
    # Fix for numpy compatibility: temporarily restore np.float if it doesn't exist
    # (needed for older matplotlib versions with newer numpy)
    # This patches numpy before matplotlib's polar projection tries to use np.float
    np_float_patched = False
    if not hasattr(np, 'float'):
        np.float = np.float64  # Use np.float64 as replacement
        np_float_patched = True
    
    try:
        # Create polar plot
        fig, ax = plt.subplots(figsize=figsize, subplot_kw=dict(projection='polar'))
        
        # Plot ranges vs angles
        ax.plot(angles, ranges, 'b-', linewidth=1, alpha=0.7, label='Lidar Scan')
        ax.scatter(angles, ranges, c=ranges, cmap='viridis', s=10, alpha=0.6)
        
        ax.set_theta_zero_location('E')  # 0 degrees at East
        ax.set_theta_direction(1)  # Counterclockwise
        ax.set_rmax(np.max(ranges) * 1.1)
        ax.set_rlabel_position(22.5)
        ax.grid(True)
        ax.set_title('Lidar Scan (Polar View)', pad=20)
        
        plt.tight_layout()
        return fig, ax
    finally:
        # Restore original state - remove the patch if we added it
        if np_float_patched and hasattr(np, 'float'):
            delattr(np, 'float')


if __name__ == '__main__':
    # Example usage
    json_file = 'cells_kernels/c4/deg0/nr_0.01_y0.46_HD0.json'
    
    # Create cartesian visualization
    print(f"Visualizing lidar scan from {json_file}")
    fig1, ax1 = visualize_lidar_scan(json_file, show_robot=True, show_rays=False)
    # plt.show()
    plt.savefig('lidar_scan_cartesian.png', dpi=300, bbox_inches='tight')
    print("Saved: lidar_scan_cartesian.png")
    plt.close(fig1)
    
    # Create polar visualization (optional, may have compatibility issues with some matplotlib versions)
    # try:
    fig2, ax2 = visualize_lidar_scan_polar(json_file)
    # plt.show()
    plt.savefig('lidar_scan_polar.png', dpi=300, bbox_inches='tight')
    print("Saved: lidar_scan_polar.png")
    
    plt.close(fig2)
    # except Exception as e:
    #     print(f"Warning: Could not create polar plot: {e}")
    #     print("Continuing with cartesian visualization only.")
    
    print("Visualization complete!")

