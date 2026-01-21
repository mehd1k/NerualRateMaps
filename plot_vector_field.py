#!/usr/bin/env python3
"""
Script to plot vector field from JSON data containing lidar readings and control vectors.
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import os
from pathlib import Path
import argparse

def plot_vector_field_from_json(json_path='trj/vector_field_data.json', 
                                 cell_ls=None, 
                                 output_path='trj/vector_field_plot.png',
                                 show_cells=True,
                                 scale_arrows=True):
    """
    Plot vector field from lidar data saved in JSON format.
    
    Args:
        json_path: Path to JSON file containing vector field data
        cell_ls: List of cell objects for plotting boundaries (optional)
        output_path: Path to save the output plot
        show_cells: Whether to plot cell boundaries
        scale_arrows: Whether to scale arrows based on magnitude
    
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
    
    # Extract positions and control vectors
    positions = []
    u_vectors = []
    headings = []
    lidar_ranges = []
    
    for entry in data:
        pos = entry.get('position', [0, 0])
        u = entry.get('u', [0, 0])
        heading = entry.get('heading', 0)
        lidar = entry.get('lidar_ranges', [])
        
        positions.append(pos)
        u_vectors.append(u)
        headings.append(heading)
        if lidar:
            lidar_ranges.append(lidar)
    
    # Convert to numpy arrays
    positions = np.array(positions)
    u_vectors = np.array(u_vectors)
    headings = np.array(headings)
    
    print(f"Position range: x=[{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}], "
          f"y=[{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
    print(f"Control vector range: u_x=[{u_vectors[:, 0].min():.3f}, {u_vectors[:, 0].max():.3f}], "
          f"u_y=[{u_vectors[:, 1].min():.3f}, {u_vectors[:, 1].max():.3f}]")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot cell boundaries if provided
    if show_cells:
        try:
            from cell_configs import cell_ls as default_cell_ls
            if cell_ls is None:
                cell_ls = default_cell_ls
            
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
    
    # Calculate arrow scaling if needed
    if scale_arrows:
        magnitudes = np.sqrt(u_vectors[:, 0]**2 + u_vectors[:, 1]**2)
        max_magnitude = np.max(magnitudes)
        min_magnitude = np.min(magnitudes[magnitudes > 0])
        if max_magnitude > 0:
            # Scale so arrows are visible but not too large
            # Use a scale that makes vectors proportional but readable
            scale = None  # Let quiver auto-scale
        else:
            scale = 1.0
        print(f"Vector magnitudes: min={min_magnitude:.4f}, max={max_magnitude:.4f}")
    else:
        scale = None
    
    # Normalize vectors for better visualization (optional)
    # magnitudes = np.sqrt(u_vectors[:, 0]**2 + u_vectors[:, 1]**2)
    # u_vectors_normalized = u_vectors / (magnitudes[:, np.newaxis] + 1e-8)
    
    # Plot vector field
    quiver = ax.quiver(positions[:, 0], positions[:, 1], 
                       u_vectors[:, 0], u_vectors[:, 1],
                       angles='xy', scale_units='xy', 
                       scale=scale,
                       width=0.003,
                       alpha=0.7,
                       color='blue',
                       headwidth=3,
                       headlength=4)
    
    # Add labels and formatting
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Vector Field from Lidar Data', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Save the plot
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Vector field plot saved to: {output_path}")
    
    return fig, ax

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot vector field from JSON data')
    parser.add_argument('--json', type=str, default='trj/vector_field_data.json',
                        help='Path to JSON file containing vector field data')
    parser.add_argument('--output', type=str, default='trj/vector_field_plot.png',
                        help='Path to save the output plot')
    parser.add_argument('--no-cells', action='store_true',
                        help='Do not plot cell boundaries')
    parser.add_argument('--no-scale', action='store_true',
                        help='Do not scale arrows')
    
    args = parser.parse_args()
    
    try:
        fig, ax = plot_vector_field_from_json(
            json_path=args.json,
            output_path=args.output,
            show_cells=not args.no_cells,
            scale_arrows=not args.no_scale
        )
        plt.show()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

