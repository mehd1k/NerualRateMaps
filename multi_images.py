"""
Created on 20240606
@author: Yanbo Lian
"""
################### Change position here ###########################
import argparse
from south_white_L import B, N, F, W, H, MAZE_ID, light_scale  # load the setup of environment
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import GeoMipTerrain, loadPrcFileData, AmbientLight
import pathlib
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat
import sys


def generate_images(grid_points, output_dir, mat_dir):
    Z = F + 20  # camera is 2cm above the floor
    P = 0  # math.atan2(H,(N-2*B)/10)*180/math.pi
    pxmm = 1

    v_peak = 130
    yaw_sd = 340
    cam_far = 3000  # see 0.8m far; 2000 for previous simulations

    x_size = (W + 1) / 1000
    y_size = (W + 1) / 1000
    VX, VY = 150, 90

    TRAIL_PLOT = False

    map_file_path = 'C:\\Users\\mahdi\\Desktop\\yanbo_code'
    map_img = map_file_path + os.sep + f"{MAZE_ID}.png"
    map_img = pathlib.PurePosixPath(pathlib.Path(map_img))
    cmap_img = map_file_path + os.sep + f"c{MAZE_ID}.png"
    cmap_img = pathlib.PurePosixPath(pathlib.Path(cmap_img))

    loadPrcFileData('', 'win-size {} {}'.format(VX, VY))
    MAZE_OUT = output_dir

    class MyApp(ShowBase):  # our 'class'
        def __init__(self):
            ShowBase.__init__(self)  # initialise

            terrain = GeoMipTerrain("ratMaze")  # create a terrain
            terrain.setHeightfield(map_img)  # set the height map
            terrain.setColorMap(cmap_img)  # set the colour map
            terrain.setBruteforce(True)
            root = terrain.getRoot()  # maximum height
            root.reparentTo(self.render)  # render from root
            root.setSz(H + F)
            terrain.generate()  # generate
            self.vismat = []
            self.camLens.setFov(VX, VY)
            self.camLens.setFar(cam_far)
            self.disableMouse()
            self.grid_points = grid_points  # List of grid points
            self.current_index = 0  # Start from the first point
            self.taskMgr.add(self.moveRat, 'moveRat')

        def moveRat(self, task):
            if self.current_index < len(self.grid_points)-1:
                # Get the current grid point
                X, Y, HD = self.grid_points[self.current_index]
                nx = np.maximum(B, np.minimum((X - x_size / 2) * 1000 + N / 2, B + W - 1))
                ny = np.maximum(B, np.minimum((Y - y_size / 2) * 1000 + N / 2, B + W - 1))
                nhd = (HD - 90) % 360

                # Update camera position and orientation
                self.camera.setPos(nx, ny, Z)
                self.camera.setHpr(nhd, P, 0)

                # Capture the screenshot and save it
                sr = self.win.getScreenshot()
                data = sr.getRamImage()
                image = np.frombuffer(data, np.uint8)
                image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
                image = np.flipud(image)

                # Save the image for the current point
                output_image_path = os.path.join(MAZE_OUT, f"x{X:.2f}_y{Y:.2f}_HD{HD:.2f}.png")
                mat_image_path = os.path.join(mat_dir, f"x{X:.2f}_y{Y:.2f}_HD{HD:.2f}")
                next_img = image[:, :, 2]
                plt.imsave(output_image_path, next_img)
                self.vismat.append(next_img.flatten())
                self.prev = next_img
                visdat = np.vstack(self.vismat)
                
                savemat(f"{mat_image_path}.mat",
                        {'images_data': next_img,
                         'positions_data': np.transpose(np.vstack((X, Y))),
                         'hds_data': HD}
                        )
                print(f"Image saved for X={X}, Y={Y}, HD={HD}")


                # Move to the next grid point
                self.current_index += 1
                return Task.cont
            else:
                self.userExit()
                # # All points processed, exit the app
                # taskMgr.doMethodLater(0.5, self.exitApp, 'exit-app')
                # return Task.done

        def exitApp(self, task):
            self.userExit()  # Close the application after finishing

    app = MyApp()
    app.run()
import shutil
def empty_dir(directory):
    ###delete all files in a directory
    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)  # Remove file or symbolic link
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)  # Remove directory and its contents
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")




def main():
    # Parse arguments from command-line
    parser = argparse.ArgumentParser(description="Generate grid points and run the image generation process.")
    
    parser.add_argument('--output_dir', type=str, required=True, help="Output directory for the images")
    parser.add_argument('--mat_dir', type=str, required=True, help="Directory for mat files")
    parser.add_argument('--num_points', type=int, default=10, help="Number of grid points along each axis")
    parser.add_argument('--x_start', type=float, default=0.01, help="Starting value for x-axis")
    parser.add_argument('--x_end', type=float, default=0.3, help="Ending value for x-axis")
    parser.add_argument('--y_start', type=float, default=0.89, help="Starting value for y-axis")
    parser.add_argument('--y_end', type=float, default=1.2, help="Ending value for y-axis")
    
    args = parser.parse_args()

    # Generate grid points based on the arguments
    x_values = np.linspace(args.x_start, args.x_end, args.num_points)
    y_values = np.linspace(args.y_start, args.y_end, args.num_points)
    HD_values = np.arange(0, 360, 10)

    x_grid, y_grid, HD_grid = np.meshgrid(x_values, y_values, HD_values, indexing='ij')
    grid_points = [(x, y, HD_value) for x, y, HD_value in zip(x_grid.flatten(), y_grid.flatten(), HD_grid.flatten())]

    # Empty directories before generating images
    empty_dir(args.output_dir)
    empty_dir(args.mat_dir)

    # Call the function to generate images
    generate_images(grid_points, args.output_dir, args.mat_dir)

if __name__ == "__main__":
    main()

# if __name__ == '__main__':
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
    
#     output_dir = 'cells_kernels\\c0\\all\\img'
#     mat_dir = 'cells_kernels\\c0\\all\\mat'
#     empty_dir(output_dir)
#     empty_dir(mat_dir)
#     # Generate images for all points in a single run
#     generate_images(grid_points, output_dir, mat_dir)

