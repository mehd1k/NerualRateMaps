import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
import subprocess

xrange_ls = [[0, 0.3], [0, 0.3], [0, 0.3], [0, 0.3], [0.3, 0.6], [0.6, 0.9], [0.9, 1.2], [0.9, 1.2], [0.6, 0.9], [0.3, 0.6], [0.3, 0.6], [0.3, 0.6]]
yrange_ls = [[0.9, 1.2], [0.6, 0.9], [0.3, 0.6], [0, 0.3], [0, 0.3], [0, 0.3], [0, 0.3], [0.3, 0.6], [0.3, 0.6], [0.3, 0.6], [0.6, 0.9], [0.9, 1.2]]

###check to see if these range are correct
def check_range(xrange_ls, yrange_ls):
    fig, ax = plt.subplots()

    # Loop through the ranges and add rectangles (squares) to the plot
    for x_range, y_range in zip(xrange_ls, yrange_ls):
        width = x_range[1] - x_range[0]
        height = y_range[1] - y_range[0]
        rect = patches.Rectangle((x_range[0], y_range[0]), width, height, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

    # Set the limits of the plot
    ax.set_xlim(min([x[0] for x in xrange_ls]) - 1, max([x[1] for x in xrange_ls]) + 1)
    ax.set_ylim(min([y[0] for y in yrange_ls]) - 1, max([y[1] for y in yrange_ls]) + 1)

    # Display the plot
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


script_path = 'multi_images.py'
for i in range (2,12):
        #     x_start, x_end = 0.01, 0.3
#     y_start, y_end = 0.89, 1.2
#     num_points = 10
#     # Generate the grid points
#     x_values =np.linspace(x_start, x_end, num_points)
#     y_values = np.linspace(y_start, y_end, num_points)
#     HD_values = np.arange(0,360,10)
   
#     # Create the grid using numpy's meshgrid function
#     x_grid, y_grid, HD_grid = np.meshgrid(x_values, y_values,HD_values,indexing='ij')
#      # Constant head direction for all points

#     # Create a list of grid points (X, Y, HD)
#     grid_points = [( x, y,HD_value ) for x, y, HD_value in zip(x_grid.flatten(), y_grid.flatten(), HD_grid.flatten())]

#     # Output directory for the images
    
    output_dir = 'cells_kernels\\c'+str(i)+'\\all\\img'
    mat_dir = 'cells_kernels\\c'+str(i)+'\\all\\mat'

    num_points = 10
    command = [
        sys.executable, script_path,
        '--output_dir', output_dir,
        '--mat_dir', mat_dir,
        '--num_points', str(num_points),
        '--x_start', str(xrange_ls[i][0]),
        '--x_end', str(xrange_ls[i][1]),
        '--y_start', str(yrange_ls[i][0]),
        '--y_end', str(yrange_ls[i][1])
    ]

    # Run the command
    subprocess.run(command)
