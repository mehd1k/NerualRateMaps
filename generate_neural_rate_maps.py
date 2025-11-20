#!/usr/bin/env python3
"""
Generate neural rate maps from JSON files.

This script reads JSON files containing neural rates and positions,
and creates rate maps for each neuron element by organizing the 
activities according to their positions on a square grid.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from typing import List, Dict, Tuple
import os


def read_json_files(input_folder: str) -> Tuple[List[Dict], int]:
    """
    Read all JSON files from the input folder.
    
    Args:
        input_folder: Path to folder containing JSON files
        
    Returns:
        List of dictionaries containing the data and number of neural elements
    """
    input_path = Path(input_folder)
    json_files = sorted(input_path.glob('*.json'))
    
    if not json_files:
        raise ValueError(f"No JSON files found in {input_folder}")
    
    data_list = []
    num_elements = None
    
    for json_file in json_files:
        with open(json_file, 'r') as f:
            data = json.load(f)
            data_list.append(data)
            
            # Determine the number of neural elements from the first file
            if num_elements is None:
                num_elements = len(data['neural_rate'])
    
    print(f"Loaded {len(data_list)} JSON files")
    print(f"Number of neural elements: {num_elements}")
    
    return data_list, num_elements


def create_neural_rate_maps(data_list: List[Dict], num_elements: int) -> np.ndarray:
    """
    Create neural rate maps for each element.
    
    Args:
        data_list: List of data dictionaries containing positions and neural rates
        num_elements: Number of neural elements
        
    Returns:
        Array of shape (num_elements, grid_height, grid_width)
    """
    # Extract all positions
    positions = [(d['position']['x'], d['position']['y']) for d in data_list]
    neural_rates = [d['neural_rate'] for d in data_list]
    
    # Determine unique x and y coordinates
    x_coords = sorted(list(set([p[0] for p in positions])))
    y_coords = sorted(list(set([p[1] for p in positions])))
    
    print(f"Grid dimensions: {len(x_coords)} x {len(y_coords)}")
    print(f"X range: {min(x_coords):.2f} to {max(x_coords):.2f}")
    print(f"Y range: {min(y_coords):.2f} to {max(y_coords):.2f}")
    
    # Create mapping from coordinates to grid indices
    x_to_idx = {x: i for i, x in enumerate(x_coords)}
    y_to_idx = {y: i for i, y in enumerate(y_coords)}
    
    # Initialize rate maps (one for each neural element)
    rate_maps = np.full((num_elements, len(y_coords), len(x_coords)), np.nan)
    
    # Fill the rate maps
    for pos, rates in zip(positions, neural_rates):
        x_idx = x_to_idx[pos[0]]
        y_idx = y_to_idx[pos[1]]
        
        for element_idx in range(num_elements):
            rate_maps[element_idx, y_idx, x_idx] = rates[element_idx]
    
    return rate_maps


def save_rate_maps_as_images(rate_maps: np.ndarray, output_folder: str, 
                              cell_id: int = None, heading: float = None):
    """
    Save each neural rate map as an image.
    
    Args:
        rate_maps: Array of shape (num_elements, grid_height, grid_width)
        output_folder: Directory to save the images
        cell_id: Cell ID (optional, for naming)
        heading: Heading direction (optional, for naming)
    """
    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)
    
    num_elements = rate_maps.shape[0]
    
    print(f"Saving {num_elements} neural rate maps to {output_folder}")
    
    for element_idx in range(num_elements):
        rate_map = rate_maps[element_idx]
        
        # Flip the map vertically so that higher y values are at the top
        rate_map_flipped = np.flipud(rate_map)
        
        # Create figure
        fig, ax = plt.subplots(figsize=(8, 8))
        
        # Plot the rate map
        im = ax.imshow(rate_map_flipped, cmap='hot', interpolation='nearest', 
                       aspect='auto', origin='lower')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Neural Rate', rotation=270, labelpad=15)
        
        # Set title
        title = f'Neural Rate Map - Element {element_idx}'
        if cell_id is not None:
            title = f'Cell {cell_id} - ' + title
        if heading is not None:
            title += f' (Heading: {heading}°)'
        ax.set_title(title)
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        
        # Save the figure
        filename = f'ratemap_element_{element_idx:03d}.png'
        output_file = output_path / filename
        plt.savefig(output_file, dpi=100, bbox_inches='tight')
        plt.close()
        
        # Also save the raw data as .npy file
        npy_filename = f'ratemap_element_{element_idx:03d}.npy'
        npy_file = output_path / npy_filename
        np.save(npy_file, rate_map)
    
    print(f"Saved {num_elements} images and .npy files")


def main():
    parser = argparse.ArgumentParser(
        description='Generate neural rate maps from JSON files'
    )
    parser.add_argument(
        'input_folder',
        type=str,
        help='Path to folder containing JSON files (e.g., cells_kernels/c0/deg0)'
    )
    parser.add_argument(
        'output_folder',
        type=str,
        help='Path to output folder for rate map images'
    )
    parser.add_argument(
        '--cell-id',
        type=int,
        default=None,
        help='Cell ID for labeling (optional)'
    )
    parser.add_argument(
        '--heading',
        type=float,
        default=None,
        help='Heading direction in degrees (optional)'
    )
    
    args = parser.parse_args()
    
    # Read JSON files
    data_list, num_elements = read_json_files(args.input_folder)
    
    # Extract cell_id and heading from first data point if not provided
    cell_id = args.cell_id if args.cell_id is not None else data_list[0].get('cell_id')
    heading = args.heading if args.heading is not None else data_list[0].get('heading')
    
    # Create neural rate maps
    rate_maps = create_neural_rate_maps(data_list, num_elements)
    
    # Save as images
    save_rate_maps_as_images(rate_maps, args.output_folder, cell_id, heading)
    
    print("Done!")


if __name__ == '__main__':
    main()

