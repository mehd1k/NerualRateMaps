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
from gen_controller import cell, Control_cal, cell_ls
from find_controller_orientation import control_gain_load
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
# MAZE_OUT =output

class MyApp(ShowBase):  # Our modified class for the control loop
    def __init__(self, initial_position, initial_hd, num_steps):
        ShowBase.__init__(self)  # Initialize graphics engine

        # Set up terrain and camera similar to the original code
        terrain = GeoMipTerrain("ratMaze")
        terrain.setHeightfield(map_img)
        terrain.setColorMap(cmap_img)
        terrain.setBruteforce(True)
        root = terrain.getRoot()
        root.reparentTo(self.render)
        root.setSz(H + F)
        terrain.generate()
        self.postion_ls = []
        self.hd_ls = []
        self.vismat = []
        self.camera.setPos(initial_position[0], initial_position[1], Z)
        self.camera.setHpr(initial_hd, P, 0)
        self.camLens.setFov(VX, VY)
        self.camLens.setFar(cam_far)
        self.disableMouse()
        self.prev = np.zeros((VY, VX))

        alight = AmbientLight('alight')
        alight.setColor((light_scale, light_scale, light_scale, 1))
        alnp = self.render.attachNewNode(alight)
        alnp.setPos(N // 2, N // 2, 2000)
        self.render.setLight(alnp)
        self.image_ls = []
        self.ratemap_ls = []

        # Store controller and movement data
        # self.controller = controller
        self.current_position = initial_position
        self.current_hd = initial_hd
        self.num_steps = num_steps
        self.current_step = 0
        self.postion_ls=[initial_position]
        self.hd_ls = [initial_hd]
        # Start MATLAB engine
        self.eng = matlab.engine.start_matlab()

        # Start the task for continuous updating
        self.taskMgr.add(self.move_and_control, 'move_and_control')
        #####Control Cells 
        self.cell_ls = cell_ls



        ####Used for finding correct control gains based on the orientation
        self.control_gain_load = control_gain_load()
        self.dt = 0.01

    def findcell(self, x):
        ls_flag=[]
        for i in range(len(self.cell_ls)):
            ls_flag.append(self.cell_ls[i].check_in_polygon(np.reshape(x,(1,2))))
        
        # print(ls_flag)
        return [i for i, x in enumerate(ls_flag) if x][0]



    def move_and_control(self, task):
        if self.current_step < self.num_steps:
            positions = [[0.01,0.51], [0.01,0.62], [0.01,0.73], [0.01,0.84], [0.01,0.95]]
            hd_ls  = [270,271,272,273,274] 
            
            self.generate_images_from_positions(positions[self.current_step], hd_ls[self.current_step])
            


          
            print(f'current step = {self.current_step}')
            # Increment step
            self.current_step += 1
            
            return Task.cont
        else:
            # Stop MATLAB engine and exit when done
          
            self.eng.quit()
            sys.exit()

    
    def generate_images_from_positions(self, pos, hd):
        """
        Generates images for a  position and heading.

        Args:
            (x, y) positions.
             headings.

        Returns:
            generated image.
        """
        
    
        nx = np.maximum(B, np.minimum((pos[0] - x_size / 2) * 1000 + N / 2, B + W - 1))
        ny = np.maximum(B, np.minimum((pos[1] - y_size / 2) * 1000 + N / 2, B + W - 1))
        nhd = (hd - 90) % 360
        self.camera.setPos(nx, ny, Z)
        self.camera.setHpr(nhd, P, 0)

        # Capture the screenshot
        sr = self.win.getScreenshot()
        data = sr.getRamImage()
        image = np.frombuffer(data, np.uint8)
        image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
        image = np.flipud(image)
        next_img = image[:, :, 2]
        plt.imsave('test/img/xyhd'+'_'+str(pos[0])+'_'+str(pos[1])+'_'+str(hd)+'.png', next_img)
        self.image_ls.append(next_img)
        # print('*******next_image_shape0', next_img.shape)
        self.vismat.append(next_img.flatten())
        self.prev = next_img
        img = np.vstack(self.vismat)
        # print('*****************************img_shape0', img.shape)
        # img = img[:, :VX * VY] / 255
        img = next_img/255
      

        return img


    def controller(self, neural_map, position, orinetation ,update_hd = 1):
        """
        Update position and head direction based on MATLAB outputs.
        """
        current_cell = self.findcell(position)
        
        K,Kb = self.control_gain_load.interpolate_contorlgains(current_cell, orinetation)
        # print('K=',K)
        # print('Kb=', Kb)
   
        

        u = K@neural_map+Kb
        print('u=',u)
        speed = 0.0
        u = u/np.linalg.norm(u)
        print('u_norm=',u)
       
        u = u*speed
        if update_hd:
            new_hd  = np.atan2(u[1], u[0])*180/np.pi
            eta = 0.5
            new_hd = ((1-eta)*orinetation+eta*float((new_hd%360)[0]))%360
            new_hd = np.sign(new_hd-orinetation)*min(np.abs(new_hd-orinetation), 30)+orinetation
        else:
            new_hd = orinetation
        print('new_hd=', new_hd )
       
       
        B_dis = np.eye(2)*self.dt
        ##heading angle in degree
       
       
        
        
        new_position = position+(B_dis@u).flatten()

        return new_position, new_hd

# Main execution
if __name__ == '__main__':
    initial_position = np.array([0.01, 0.5])
    initial_hd = 270
    num_steps = 5

    app = MyApp( initial_position, initial_hd, num_steps)
    app.run()
