import json
from re import A
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Patch
from matplotlib.colors import ListedColormap


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


def generate_occupancy_grid(json_file, grid_size=10, front_angle_range=(-np.pi/3, np.pi/3) ):
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
    data = load_lidar_data(json_file)
    lidar_scan = data['lidar_scan']
    
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


def generate_occupancy_grid_polar(json_file, num_angle_bins=120, num_range_bins=40):
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
    data = load_lidar_data(json_file)
    lidar_scan = data['lidar_scan']
    
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


def visualize_occupancy_grid_polar(occupancy_grid, polar_params=None, figsize=(12, 10), show_as_image=False):
    """
    Visualize the polar occupancy grid map.
    
    Args:
        occupancy_grid: 2D numpy array of shape (num_range_bins, num_angle_bins) with occupancy values
                       (1.0=occupied, 0.0=free, 0.5=unknown)
        polar_params: Optional dictionary with polar grid parameters. If None, will infer from grid shape.
        figsize: Figure size (width, height)
        show_as_image: If True, shows as rectangular image (angle vs range). If False, shows as polar plot.
    
    Returns:
        fig, ax: Figure and axes objects
    """
    num_range_bins, num_angle_bins = occupancy_grid.shape
    
    # Prepare polar parameters
    if polar_params is None:
        # Infer from grid shape
        angle_bin_increment = 3.0 * np.pi / 180.0  # 3 degrees in radians
        angle_grid_min = 0.0
        angle_grid_max = num_angle_bins * angle_bin_increment
        range_min = 0.0
        range_max = 1.2
        range_increment = range_max / num_range_bins
    else:
        angle_grid_min = polar_params.get('angle_min', 0.0)
        angle_grid_max = polar_params.get('angle_max', 2 * np.pi)
        angle_bin_increment = polar_params.get('angle_increment', 3.0 * np.pi / 180.0)
        range_min = polar_params.get('range_min', 0.0)
        range_max = polar_params.get('range_max', 1.2)
        range_increment = polar_params.get('range_increment', range_max / num_range_bins)
    range_max = 1.2
    if show_as_image:
        # Show as rectangular image (angle vs range)
        fig, ax = plt.subplots(figsize=figsize)
        
        # Create angle and range arrays for display
        angles = np.linspace(angle_grid_min, angle_grid_max, num_angle_bins, endpoint=False)
        ranges = np.linspace(range_min, range_max, num_range_bins, endpoint=False)
        
        # Create meshgrid for pcolormesh
        ANG, RNG = np.meshgrid(angles, ranges)
        
        # Create custom colormap: white for free, gray for unknown, black for occupied
        colors = ['white', 'gray', 'black']  # Free (0.0), Unknown (0.5), Occupied (1.0)
        cmap = ListedColormap(colors)
        
        # Display the grid
        im = ax.pcolormesh(ANG, RNG, occupancy_grid, cmap=cmap, vmin=0.0, vmax=1.0, shading='auto')
        
        # Convert angles to degrees for display
        ax.set_xlabel('Angle (degrees)')
        ax.set_ylabel('Range (m)')
        ax.set_title(f'Polar Occupancy Grid ({num_range_bins}x{num_angle_bins})')
        
        # Set x-axis to show degrees
        angle_ticks_deg = np.linspace(0, 360, 9)  # 0, 45, 90, ..., 360
        angle_ticks_rad = np.deg2rad(angle_ticks_deg)
        ax.set_xticks(angle_ticks_rad)
        ax.set_xticklabels([f'{int(d)}°' for d in angle_ticks_deg])
        
        ax.grid(True, alpha=0.3)
        
        # Create custom legend
        legend_elements = [
            Patch(facecolor='white', edgecolor='black', label='Free (0.0)'),
            Patch(facecolor='gray', edgecolor='black', label='Unknown (0.5)'),
            Patch(facecolor='black', edgecolor='black', label='Occupied (1.0)')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
    else:
        # Show as polar plot
        # Fix for numpy compatibility: temporarily restore np.float if it doesn't exist
        np_float_patched = False
        if not hasattr(np, 'float'):
            np.float = np.float64
            np_float_patched = True
        
        try:
            fig, ax = plt.subplots(figsize=figsize, subplot_kw=dict(projection='polar'))
            
            # Create angle and range arrays
            angles = np.linspace(angle_grid_min, angle_grid_max, num_angle_bins, endpoint=False)
            ranges = np.linspace(range_min, range_max, num_range_bins, endpoint=False)
            
            # Create meshgrid for pcolormesh
            ANG, RNG = np.meshgrid(angles, ranges)
            
            # Create custom colormap: white for free, gray for unknown, black for occupied
            colors = ['white', 'gray', 'black']  # Free (0.0), Unknown (0.5), Occupied (1.0)
            cmap = ListedColormap(colors)
            
            # Display the grid
            im = ax.pcolormesh(ANG, RNG, occupancy_grid, cmap=cmap, vmin=0.0, vmax=1.0, shading='auto')
            
            # Configure polar plot
            ax.set_theta_zero_location('E')  # 0 degrees at East
            ax.set_theta_direction(1)  # Counterclockwise
            ax.set_rmax(range_max)
            ax.set_rlabel_position(22.5)
            ax.grid(True)
            ax.set_title(f'Polar Occupancy Grid ({num_range_bins}x{num_angle_bins})', pad=20)
            
            # Create custom legend
            legend_elements = [
                Patch(facecolor='white', edgecolor='black', label='Free (0.0)'),
                Patch(facecolor='gray', edgecolor='black', label='Unknown (0.5)'),
                Patch(facecolor='black', edgecolor='black', label='Occupied (1.0)')
            ]
            ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.3, 1.0))
            
        finally:
            # Restore original state
            if np_float_patched and hasattr(np, 'float'):
                delattr(np, 'float')
    
    plt.tight_layout()
    return fig, ax


def visualize_occupancy_grid(occupancy_grid, bounds=None, show_robot=True, figsize=(10, 10)):
    """
    Visualize the occupancy grid map.
    
    Args:
        occupancy_grid: 2D numpy array with occupancy values (1.0=occupied, 0.0=free, 0.5=unknown)
        bounds: Optional dictionary with grid bounds (x_min, x_max, y_min, y_max, cell_width, cell_height)
        show_robot: Whether to show robot position marker
        figsize: Figure size (width, height)
    
    Returns:
        fig, ax: Figure and axes objects
    """
    # Create figure
    fig, ax = plt.subplots(figsize=figsize)
    
    # Determine grid size
    grid_size = occupancy_grid.shape[0]
    
    # Prepare bounds for display
    if bounds is None:
        # Default bounds if not provided
        x_min, x_max = -5.0, 5.0
        y_min, y_max = -5.0, 5.0
    else:
        x_min = bounds.get('x_min', -5.0)
        x_max = bounds.get('x_max', 5.0)
        y_min = bounds.get('y_min', -5.0)
        y_max = bounds.get('y_max', 5.0)
    
    # Display the occupancy grid as an image
    # Flip vertically because image origin is at top-left, but we want y-axis pointing up
    grid_display = np.flipud(occupancy_grid)
    
    # Create custom colormap: black for occupied, white for free, gray for unknown
    colors = ['white', 'gray', 'black']  # Free (0.0), Unknown (0.5), Occupied (1.0)
    cmap = ListedColormap(colors)
    
    # Display the grid
    im = ax.imshow(grid_display, cmap=cmap, aspect='equal', 
                   extent=[x_min, x_max, y_min, y_max],
                   interpolation='nearest', vmin=0.0, vmax=1.0)
    
    # Add grid lines to show cell boundaries
    cell_width = (x_max - x_min) / grid_size
    cell_height = (y_max - y_min) / grid_size
    
    # Vertical lines
    for i in range(grid_size + 1):
        x_line = x_min + i * cell_width
        ax.axvline(x_line, color='lightgray', linewidth=0.5, alpha=0.5)
    
    # Horizontal lines
    for i in range(grid_size + 1):
        y_line = y_min + i * cell_height
        ax.axhline(y_line, color='lightgray', linewidth=0.5, alpha=0.5)
    
    # Show robot position (assumed at origin)
    if show_robot:
        robot_x = 0.0
        robot_y = 0.0
        ax.plot(robot_x, robot_y, 'ro', markersize=12, label='Robot Position', 
                markeredgecolor='darkred', markeredgewidth=2, zorder=10)
        
        # Draw a circle around robot
        circle = Circle((robot_x, robot_y), min(cell_width, cell_height) * 0.3, 
                       fill=False, color='red', linestyle='--', linewidth=2, 
                       alpha=0.7, zorder=9)
        ax.add_patch(circle)
    
    # Add text annotations for cell values (optional, can be disabled for large grids)
    if grid_size <= 20:  # Only show text for reasonably sized grids
        for row in range(grid_size):
            for col in range(grid_size):
                value = occupancy_grid[row, col]
                # Convert grid indices to world coordinates for text placement
                world_x = x_min + (col + 0.5) * cell_width
                world_y = y_max - (row + 0.5) * cell_height  # Flip y-coordinate
                
                # Determine text color based on cell value
                text_color = 'black' if value < 0.75 else 'white'
                
                # Show value as text
                ax.text(world_x, world_y, f'{value:.1f}', 
                       ha='center', va='center', fontsize=8,
                       color=text_color, weight='bold')
    
    # Labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Occupancy Grid Map ({grid_size}x{grid_size})')
    ax.grid(False)  # We're using custom grid lines
    
    # Create custom legend for occupancy values
    legend_elements = [
        Patch(facecolor='white', edgecolor='black', label='Free (0.0)'),
        Patch(facecolor='gray', edgecolor='black', label='Unknown (0.5)'),
        Patch(facecolor='black', edgecolor='black', label='Occupied (1.0)')
    ]
    if show_robot:
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', 
                                         markerfacecolor='red', markersize=10,
                                         markeredgecolor='darkred', markeredgewidth=2,
                                         label='Robot Position'))
    
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Set equal aspect ratio
    ax.set_aspect('equal')
    
    plt.tight_layout()
    return fig, ax


def visualize_rsc_response(cell_index, figsize = (12, 10)):
    import os
    directory = f'/home/mehdi/NerualRateMaps/allocentric_ratemaps/RSC/data'
    
    file_name = 'cell_'+str(cell_index)+'.npy'
    file_path = os.path.join(directory, file_name)
    array = np.load(file_path)
    
   
    num_angle_bins, num_range_bins = array.shape
    
   
    # Infer from grid shape
    angle_bin_increment = 3.0 * np.pi / 180.0  # 3 degrees in radians
    angle_grid_min = 0.0
    angle_grid_max = num_angle_bins * angle_bin_increment
    range_min = 0.0
    range_max = 1.2
    range_increment = range_max / num_range_bins


    # Show as polar plot
    # Fix for numpy compatibility: temporarily restore np.float if it doesn't exist
    np_float_patched = False
    if not hasattr(np, 'float'):
        np.float = np.float64
        np_float_patched = True
    
    try:
        fig, ax = plt.subplots(figsize=figsize, subplot_kw=dict(projection='polar'))
        
        # Create angle and range arrays
        angles = np.linspace(angle_grid_min, angle_grid_max, num_angle_bins, endpoint=False)
        ranges = np.linspace(range_min, range_max, num_range_bins, endpoint=False)
        
        # Create meshgrid for pcolormesh
        RNG, ANG = np.meshgrid(ranges, angles)
        
        # Create custom colormap: white for free, gray for unknown, black for occupied
        # colors = ['white', 'gray', 'black']  # Free (0.0), Unknown (0.5), Occupied (1.0)
        # cmap = ListedColormap(colors)
        
        # Display the grid
        im = ax.pcolormesh(ANG, RNG, array, shading='auto', cmap='viridis')
        
        # Configure polar plot
        # ax.set_theta_zero_location('E')  # 0 degrees at East
        # ax.set_theta_direction(1)  # Counterclockwise
        # ax.set_rmax(range_max)
        # ax.set_rlabel_position(22.5)
        # ax.grid(True)
        # ax.set_title(f'Polar Occupancy Grid ({num_range_bins}x{num_angle_bins})', pad=20)
        
        # # Create custom legend
        # legend_elements = [
        #     Patch(facecolor='white', edgecolor='black', label='Free (0.0)'),
        #     Patch(facecolor='gray', edgecolor='black', label='Unknown (0.5)'),
        #     Patch(facecolor='black', edgecolor='black', label='Occupied (1.0)')
        # ]
        # ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.3, 1.0))
        
    finally:
        # Restore original state
        if np_float_patched and hasattr(np, 'float'):
            delattr(np, 'float')
    
    plt.tight_layout()
    return fig, ax

if __name__ == '__main__':
    # Example usage
    json_file = 'cells_kernels/c4/deg90/nr_0.01_y0.46_HD90.json'
    
    # # Create cartesian visualization
    # print(f"Visualizing lidar scan from {json_file}")
    # fig1, ax1 = visualize_lidar_scan(json_file, show_robot=True, show_rays=False)
    # # plt.show()
    # plt.savefig('lidar_scan_cartesian.png', dpi=300, bbox_inches='tight')
   

    # # Generate the grid
    # occupancy_grid, bounds = generate_occupancy_grid(json_file)

    # # Visualize it
    # fig, ax = visualize_occupancy_grid(occupancy_grid, bounds, show_robot=True)
    # # plt.show()
    # plt.savefig('occupancy_grid.png', dpi=300, bbox_inches='tight')



    # Generate the polar grid
    occupancy_grid_polar, polar_params = generate_occupancy_grid_polar(json_file)

    ##Visualize as polar plot (default)
    fig, ax = visualize_occupancy_grid_polar(occupancy_grid_polar, polar_params)
    # plt.show()
    plt.savefig('polar_occupancy_grid.png', dpi=300, bbox_inches='tight')

    # fig, ax = visualize_rsc_response(7)
    # plt.show()
    # plt.savefig('rsc_response.png', dpi=300, bbox_inches='tight')
    
    # Create polar visualization (optional, may have compatibility issues with some matplotlib versions)
    # try:
    # fig2, ax2 = visualize_lidar_scan_polar(json_file)
    # # plt.show()
    # plt.savefig('lidar_scan_polar.png', dpi=300, bbox_inches='tight')
    # print("Saved: lidar_scan_polar.png")
    
    # plt.close(fig2)
    # except Exception as e:
    #     print(f"Warning: Could not create polar plot: {e}")
    #     print("Continuing with cartesian visualization only.")
    
    print("Visualization complete!")

