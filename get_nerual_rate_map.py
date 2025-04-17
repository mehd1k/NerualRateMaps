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
import matlab.engine



def parse_arguments():
    parser = argparse.ArgumentParser(description="Generate images based on specified positions and head directions.")
    parser.add_argument("--X", type=float, required=True, help="X position in meters")
    parser.add_argument("--Y", type=float, required=True, help="Y position in meters")
    parser.add_argument("--HD", type=float, required=True, help="Head direction in degrees")
    parser.add_argument("--output", type=str, required=True, help="Output image file path")
    return parser.parse_args()

def generate_images(X, Y, HD, output):
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
    MAZE_OUT =output

    # The current position and head direction
    n = 2
    y = Y * np.ones((n,))  # unit: m
    x = X * np.ones((n,))
    hd = HD * np.ones((n,))

    # Transform input trajectory into Panda3D reference
    nx = np.maximum(B, np.minimum((x - x_size / 2) * 1000 + N / 2, B + W - 1))
    ny = np.maximum(B, np.minimum((y - y_size / 2) * 1000 + N / 2, B + W - 1))
    nhd = (hd - 90) % 360

    Nsteps = n

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
            self.camera.setPos(nx[0], ny[0], Z)
            self.camera.setHpr(nhd[0], P, 0)
            self.camLens.setFov(VX, VY)
            self.camLens.setFar(cam_far)
            self.disableMouse()
            self.taskMgr.add(self.moveRat, 'moveRat')
            self.prev = np.zeros((VY, VX))
            self.hpr = []
            self.pos = []
            if TRAIL_PLOT:
                self.ax = plt.axes()
                plt.xlim(0, N)
                plt.ylim(0, N)

            alight = AmbientLight('alight')
            alight.setColor((light_scale, light_scale, light_scale, 1))
            alnp = self.render.attachNewNode(alight)
            alnp.setPos(N // 2, N // 2, 2000)
            self.render.setLight(alnp)

        def moveRat(self, task):
            self.camera.setPos(nx[task.frame], ny[task.frame], Z)
            self.camera.setHpr(nhd[task.frame], P, 0)
            self.pos.append(self.camera.getPos())
            self.hpr.append([hd[task.frame], P, 0])
            sr = self.win.getScreenshot()
            data = sr.getRamImage()  # use data.get_data() instead of data in python 2
            image = np.frombuffer(data, np.uint8)
            image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
            image = np.flipud(image)

            if task.frame < Nsteps - 1:
                next_img = image[:, :, 2]
                self.prev = next_img
                return Task.cont
            else:
                next_img = image[:, :, 2]
                # plt.imsave(output+'.png', next_img)
                self.vismat.append(next_img.flatten())
                self.prev = next_img
                visdat = np.vstack(self.vismat)
                img = visdat[:, :VX * VY]/255
                print('image_shape', img.shape)
                print('img',img)
                # Start the MATLAB engine
                eng = matlab.engine.start_matlab()
                # S = np.zeros([100,1])
                # U = np.zeros([100,1])
                S = np.zeros([100,1])
                U = np.zeros([100,1])
                img_matlab = matlab.double(img.tolist())  # If img is a numpy array, use tolist() to convert
                S_matlab = matlab.double(S.tolist())  # Same for S
                U_matlab = matlab.double(U.tolist())  # Same for U
                img_double = eng.double(img)
                # S_out = eng.generate_V1_RSC_model_response3(img_double, nargout=1)
                S_out, U_out = eng.generate_V1_RSC_model_response(img, S, U, nargout=2)

                # Process the outputs in Python as needed
                
                print('Sout',S_out)
                print('S_out_size',np.array(S_out).shape)
                


                # Stop the MATLAB engine
                eng.quit()
                # savemat(f"{MAZE_OUT}.mat",
                #         {'images_data': visdat[:, :VX * VY],
                #          'positions_data': np.transpose(np.vstack((X, Y))),
                #          'hds_data': HD}
                #         )
                # plt.imsave(save_dir_png, visdat[:, :VX * VY])
                taskMgr.doMethodLater(0.5, self.exitApp, 'exit-app')
                return Task.done

        def exitApp(self, task):
            sys.exit()

    app = MyApp()
    app.run()

# if __name__ == '__main__':
#     args = parse_arguments()
#     generate_images(args.X, args.Y, args.HD, args.output)

if __name__ == '__main__':
    x= 0.1
    y=0.5
    HD_value = 270
    generated_image_path_mat = 'test/test'
    generate_images(x, y, HD_value, generated_image_path_mat)
    # x_start, x_end, x_increment = 0, 0.3, 0.1
    # y_start, y_end, y_increment = 0.9, 1.2, 0.1

    # # Generate the grid points
    # x_values = np.arange(x_start, x_end + x_increment, x_increment)
    # y_values = np.arange(y_start, y_end + y_increment, y_increment)

    # # Create the grid using numpy's meshgrid function
    # x_grid, y_grid = np.meshgrid(x_values, y_values)
    # counter = 0
    
    # # Print the grid points
    # for x, y in zip(x_grid.flatten(), y_grid.flatten()):
    #     print('Processing counter ===========', counter)
    #     counter += 1
        
    #     generated_image_path_mat = f'cells_kernels\\c0\\deg0\\mat\\x{x}y{y}'
    #     generated_image_path_png = f'cells_kernels\\c0\\deg0\\img\\x{x}y{y}'
    #     HD_value = 0

    #     # Step 1: Generate the image (mat file)
    #     generate_images(x, y, HD_value, generated_image_path_mat)