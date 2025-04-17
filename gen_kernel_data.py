import subprocess
import sys
import numpy as np

import matplotlib.pyplot as plt
import matlab.engine
from multi_images import generate_images, empty_dir
import atexit
import argparse



def save_img_smaple(x_start, x_end, y_start, y_end, HD_value, output_dir,mat_dir, num_points = 10 ):  
   
    # Generate the grid points
    x_values =np.linspace(x_start, x_end, num_points)
    y_values = np.linspace(y_start, y_end, num_points)
    # Create the grid using numpy's meshgrid function
    x_grid, y_grid = np.meshgrid(x_values, y_values)
    

    # Create a list of grid points (X, Y, HD)
    grid_points = [(x, y, HD_value) for x, y in zip(x_grid.flatten(), y_grid.flatten())]

    # Output directory for the images

   
    empty_dir(output_dir)
    empty_dir(mat_dir)
    # Generate images for all points in a single run
    generate_images(grid_points, output_dir, mat_dir)


if __name__ == "__main__":
    x_start, x_end = 0.01, 0.3
    y_start, y_end = 0.89, 1.2
    num_points = 10
    HD_value = 0  # Constant head direction for all points
    output_dir = 'cells_kernels\\c0\\deg0\\img'
    mat_dir = 'cells_kernels\\c0\\deg0\\mat'

