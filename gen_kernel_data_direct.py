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
    def __init__(self, cell_id):
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
        self.cell_id = cell_id
        self.cell_ls = cell_ls
        self.gen_grid_points()
        self.current_step = 0
        initial_position = np.array([self.X_all[0], self.Y_all[0]])
        nx = np.maximum(B, np.minimum((self.X_all[self.current_step] - x_size / 2) * 1000 + N / 2, B + W - 1))
        ny = np.maximum(B, np.minimum((self.Y_all[self.current_step] - y_size / 2) * 1000 + N / 2, B + W - 1))
        nhd = (self.HD_all[self.current_step] - 90) % 360
        self.camera.setPos(nx, ny , Z)
        self.camera.setHpr(nhd, P, 0)
        self.camLens.setFov(VX, VY)
        self.camLens.setFar(cam_far)
        self.disableMouse()
        self.prev = np.zeros((VY, VX))

        alight = AmbientLight('alight')
        alight.setColor((light_scale, light_scale, light_scale, 1))
        alnp = self.render.attachNewNode(alight)
        alnp.setPos(N // 2, N // 2, 2000)
        self.render.setLight(alnp)

        # Store controller and movement data
        # self.controller = controller
       
        self.num_steps = len(self.X_all)
        
        self.ux_ls = []
        self.uy_ls = []
        
        # Start MATLAB engine
        self.eng = matlab.engine.start_matlab()

        # Start the task for continuous updating
        self.taskMgr.add(self.gen_neural_rate_map, 'gen_neural_rate_map')
        #####Control Cells 
        self.cell_ls = cell_ls
        self.init_flage = True



        ####Used for finding correct control gains based on the orientation
        self.control_gain_load = control_gain_load()
        self.dt = 0.01

    def findcell(self, x):
        ls_flag=[]
        for i in range(len(self.cell_ls)):
            ls_flag.append(self.cell_ls[i].check_in_polygon(np.reshape(x,(1,2))))
        
        # print(ls_flag)
        return [i for i, x in enumerate(ls_flag) if x][0]
    
    def gen_grid_points(self, num_points = 10):

        ####First grid
        delta = 0.01
        self.xmin = np.min(self.cell_ls[self.cell_id].vrt[:,0])+delta
        self.ymin = np.min(self.cell_ls[self.cell_id].vrt[:,1])+delta
        self.xmax = np.max(self.cell_ls[self.cell_id].vrt[:,0])-delta
        self.ymax = np.max(self.cell_ls[self.cell_id].vrt[:,1])-delta
       
        x_range0  = np.linspace(self.xmin, self.xmax,num_points)
        y_range0 = np.linspace(self.ymin, self.ymax,num_points)
        hd_range = np.arange(0,360,10)
        self.X_all, self.Y_all, self.HD_all = np.meshgrid(x_range0, y_range0, hd_range)
        self.X_all, self.Y_all, self.HD_all = self.X_all.flatten(), self.Y_all.flatten(), self.HD_all.flatten()

       




    def gen_neural_rate_map(self, task):
        if self.current_step < self.num_steps:
            # Move the camera to the new position
            file_name = (
                'matx' + f"{self.X_all[self.current_step]:.2f}" +
                '_y' + f"{self.Y_all[self.current_step]:.2f}" +
                '_HD' + f"{self.HD_all[self.current_step]:.2f}"
            )
            image_path = os.path.join('cells_kernels_images', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
            s_path =  os.path.join('cells_kernels', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
            nx = np.maximum(B, np.minimum((self.X_all[self.current_step] - x_size / 2) * 1000 + N / 2, B + W - 1))
            ny = np.maximum(B, np.minimum((self.Y_all[self.current_step] - y_size / 2) * 1000 + N / 2, B + W - 1))
            nhd = (self.HD_all[self.current_step] - 90) % 360
            self.camera.setPos(nx, ny , Z)
            self.camera.setHpr(nhd, P, 0)
            print('current_step', self.current_step, '/'+str(len(self.X_all)))
            print('camera_position, hd', self.X_all[self.current_step], self.Y_all[self.current_step], self.HD_all[self.current_step])

            # Capture the screenshot
            sr = self.win.getScreenshot()
            data = sr.getRamImage()
            image = np.frombuffer(data, np.uint8)
            image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
            image = np.flipud(image)
            # print('*******image_shape0', image.shape)
            next_img = image[:, :, 2]
            plt.imsave(os.path.join(image_path,file_name)+'.png', next_img)
            # print('*******next_image_shape0', next_img.shape)
            self.vismat.append(next_img.flatten())
            self.prev = next_img
            img = np.vstack(self.vismat)
            # print('*****************************img_shape0', img.shape)
            # img = img[:, :VX * VY] / 255
            img = next_img/255
           


            S = np.zeros([100,1])
            U = np.zeros([100,1])
           
            S_out, U_out = self.eng.generate_V1_RSC_model_response(img, S, U, nargout=2)
            # print('S_out',S_out)
            np.save(os.path.join(s_path,file_name), S_out)
            # Update position and head direction using the controller
            
            if self.init_flage:
                self.init_flage =False
            else: 
                self.current_step += 1
            return Task.cont
        else:
            # Stop MATLAB engine and exit when done
            # np.save('trj/ux_ls.npy',  self.ux_ls)
            # np.save('trj/uy_ls.npy',  self.uy_ls)
            print('completed:')
            # self.vec_field_save()
            self.eng.quit()
            sys.exit()
    

    

    def vec_field_save(self):
        fig, ax = plt.subplots()
        bars = [[[0, 1.2],[0, 0], [1.2, 0], [1.2 ,0.6], [0.6, 0.6], [0.6, 1.2]] ]

        ###Ploting cells
        for cell in self.cell_ls:
            num_vrt = len(cell.vrt)-1
            for i in range(num_vrt):
                ax.plot([cell.vrt[i][0], cell.vrt[i+1][0]], [cell.vrt[i][1], cell.vrt[i+1][1]], color = 'gray')

            ax.plot([cell.vrt[0][0], cell.vrt[-1][0]], [cell.vrt[0][1], cell.vrt[-1][1]], color = 'gray')
       
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        ax.set_aspect('equal')
        ax.quiver(self.X_all, self.Y_all, self.ux_ls, self.uy_ls, angles='xy', scale_units='xy')
        fig.savefig('trj/vec_all.png', dpi= 350)


    def controller(self, neural_map, position, orinetation):
        """
        Update position and head direction based on MATLAB outputs.
        """
        current_cell = self.findcell(position)
        
        K,Kb = self.control_gain_load.interpolate_contorlgains(current_cell, orinetation)
   
        

        u = K@neural_map+Kb
       
        
        
       
        u = u/np.linalg.norm(u)

        return u[0], u[1]

# Main execution
if __name__ == '__main__':
    
 

    app = MyApp(cell_id= 11)
    app.run()
