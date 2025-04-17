import subprocess
import sys
import numpy as np

def call_image_generation_script(X, Y, HD, output):
    script_path = 'modified_script.py'
    subprocess.run([sys.executable, script_path, "--X", str(X), "--Y", str(Y), "--HD", str(HD), "--output", output])

if __name__ == "__main__":
    # # Example lists for X, Y, and HD
    # traj_ls = np.load('traj_ls.npy')
    # X_ls = traj_ls[0,:]*1.2/40
    # Y_ls = traj_ls[1,:]*1.2/40
    # HD_ls = [270]
    # for i in range(len(X_ls)-1):
    #     alpha  = np.rad2deg(np.arctan2(Y_ls[i+1]-Y_ls[i], X_ls[i+1]-X_ls[i]))
    #     if alpha < 0:
    #         alpha = alpha +360
    #     HD_ls.append(alpha)
    # print('size of traj:', len(X_ls))

    # for i, (X_value, Y_value, HD_value) in enumerate(zip(X_ls, Y_ls, HD_ls)):
    #     output_image_path = f'indx_{i}.png'
    #     call_image_generation_script(X_value, Y_value, HD_value, output_image_path)
    x_start, x_end, x_increment = 0, 0.3, 0.1
    y_start, y_end, y_increment = 0.9, 1.2, 0.1

    # Generate the grid points
    x_values = np.arange(x_start, x_end + x_increment, x_increment)
    y_values = np.arange(y_start, y_end + y_increment, y_increment)

    # Create the grid using numpy's meshgrid function
    x_grid, y_grid = np.meshgrid(x_values, y_values)
    counter = 0
    for x, y in zip(x_grid.flatten(), y_grid.flatten()):
        
        # X_value, Y_value, HD_value = 0.0, 0.0,270
        # print('counter ===========',counter)
        # counter +=1 
        # print(f"x: {x}, y: {y}")

        generated_image_path_mat = 'cells_kernels\\c0\\deg0\\mat\\x'+str(x)+'y'+str(y)
        generated_image_path_png = 'cells_kernels\\c0\\deg0\\img\\x'+str(x)+'y'+str(y)
        HD_value = 0 
        print(generated_image_path_mat)
        call_image_generation_script(x, y, HD_value, generated_image_path_mat)