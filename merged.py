"""
Created on 20240606
@author: Yanbo Lian
"""
################### Change position here ###########################
import argparse
import json
from south_white_L import B, N, F, W, H, MAZE_ID, light_scale  # load the setup of environment
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import GeoMipTerrain, loadPrcFileData, AmbientLight, loadPrcFileData
from panda3d.core import WindowProperties
import pathlib
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat
import sys
import matlab.engine
from gen_controller import cell, Control_cal, cell_ls
import cv2
from find_controller_orientation import control_gain_load
import argparse
Z = F + 20  # camera is 2cm above the floor
P = 0  # math.atan2(H,(N-2*B)/10)*180/math.pi
pxmm = 1

v_peak = 130
yaw_sd = 340
sc_img = 1
cam_far = int(3000/sc_img)  # see 0.8m far; 2000 for previous simulations

x_size = (W + 1) / 1000
y_size = (W + 1) / 1000
VX, VY = int(150*sc_img), int(90*sc_img)

TRAIL_PLOT = False
map_file_path = '/home/mker/yanbo_code'
map_img = map_file_path + os.sep + f"{MAZE_ID}.png"
map_img = pathlib.PurePosixPath(pathlib.Path(map_img))
cmap_img = map_file_path + os.sep + f"c{MAZE_ID}.png"
cmap_img = pathlib.PurePosixPath(pathlib.Path(cmap_img))
resolution_factor=10
loadPrcFileData('', 'win-size {} {}'.format(int(VX*resolution_factor), int(VY*resolution_factor)))
# MAZE_OUT =output

class MyApp(ShowBase):  # Our modified class for the control loop
    def __init__(self, task, run_flag =1):
        ShowBase.__init__(self)  # Initialize graphics engine
        properties = WindowProperties()
        resolution_factor=10
        properties.setSize(int(VX*resolution_factor), int(VY*resolution_factor))
        self.win.requestProperties(properties)

       
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
        # self.camera.setPos(initial_position[0], initial_position[1], Z)
        # self.camera.setHpr(initial_hd, P, 0)
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
        self.task = task
        # Store controller and movement data
        # self.controller = controller
        
        # Start MATLAB engine
        self.eng = matlab.engine.start_matlab()
        self.init_flage = True
        self.current_step = 0
        # Start the task for continuous updating
        
        #####Control Cells 
        self.cell_ls = cell_ls
        ####Used for finding correct control gains based on the orientation
        self.control_gain_load = control_gain_load()
        self.dt = 0.01
        ###gen_data_task
       
            
        if run_flag:
            self.taskMgr.add(self.run_me, 'run_me')
    
    def vertices(self):
        delta = 0.01
        self.xmin = np.min(self.cell_ls[self.cell_id].vrt[:,0])+delta
        self.ymin = np.min(self.cell_ls[self.cell_id].vrt[:,1])+delta
        self.xmax = np.max(self.cell_ls[self.cell_id].vrt[:,0])-delta
        self.ymax = np.max(self.cell_ls[self.cell_id].vrt[:,1])-delta

    def sample_2d_segment(self, num_samples = 50, vec_field = 0) :
       
        self.vertices()
        start_point = np.array([self.xmin,self.ymin])
        end_point = np.array([self.xmax, self.ymax])
        
        if vec_field:
            num_samples = 10
            x_range0  = np.round( np.linspace(self.xmin, self.xmax,num_samples),2)
            y_range0 = np.round( np.linspace(self.ymin, self.ymax,num_samples),2)
        
            
            self.X_all, self.Y_all = np.meshgrid( x_range0, y_range0)
            self.X_all, self.Y_all = self.X_all.flatten(), self.Y_all.flatten()
            self.HD_all = np.ones(num_samples**2)*self.hd
        else:
            self.X_all = np.ones(num_samples)*0.18
            self.Y_all = np.linspace(np.round(self.ymin,2), np.round(self.ymax,2), num_samples)
            self.HD_all = np.ones(num_samples)*self.hd

            
            # self.X_all = np.linspace(start_point[0], end_point[0], num_samples)
            # self.Y_all = np.linspace(start_point[1], end_point[1], num_samples)
            # self.HD_all = np.ones(num_samples)*self.hd


       
        
       
        
       
  
    def gen_grid_points(self, num_points = 10):

        ####First grid
        
        self.vertices()
        
        # x_range0  = np.arange(self.xmin, self.xmax, 0.01)
        # y_range0 = np.arange(self.ymin, self.ymax, 0.01)

        x_range0  = np.round( np.linspace(self.xmin, self.xmax,num_points),2)
        y_range0 = np.round( np.linspace(self.ymin, self.ymax,num_points),2)
        hd_range = np.arange(0,360,10)
        # self.X_all, self.Y_all, self.HD_all = np.meshgrid(x_range0, y_range0, hd_range)
        self.HD_all,self.X_all, self.Y_all = np.meshgrid(hd_range, x_range0, y_range0)
        self.X_all, self.Y_all, self.HD_all = self.X_all.flatten(), self.Y_all.flatten(), self.HD_all.flatten()
    
    
    def findcell(self, x):
        ls_flag=[]
        for i in range(len(self.cell_ls)):
            ls_flag.append(self.cell_ls[i].check_in_polygon(np.reshape(x,(1,2))))
        
        # print(ls_flag)
        return [i for i, x in enumerate(ls_flag) if x][0]
    

    def init_gen_data(self, cell_id):
        self.cell_id = cell_id
        self.gen_grid_points()
        self.num_steps = len(self.X_all)

    def init_lipt_test_line(self, cell_id, hd, vec_field = 0):
        self.cell_id = cell_id
        self.hd = hd
        
        self.pos_ls = []
        self.img_ls = []
        self.nr_ls = []
        self.u_ls = []
        self.hd_ls =[]
        self.vec_field = vec_field
       
        self.sample_2d_segment(vec_field = vec_field) 
      
            
        # self.gen_grid_points()
        self.num_steps = len(self.X_all)

    

    def init_lipt_test_orientation(self, pos):
        
        self.pos = pos
        self.cell_id = self.findcell(pos)
        print(f'*****************cell   {self.cell_id}')
        self.img_ls = []
        self.nr_ls = []
        self.u_ls = []
        self.HD_all = range(360)
        self.num_steps = len(self.HD_all)


    def init_implementation(self, init_pos, init_hd, num_steps):
        self.image_ls = []
        self.postion_ls = []
        self.hd_ls = []
        self.ratemap_ls = []
        self.u_ls =[]
        self.current_position = init_pos
        self.current_hd = init_hd
        self.num_steps = num_steps




    def run_me(self, task):
        if self.current_step < self.num_steps:


            # positions = [[0.01,0.51], [0.01,0.62], [0.01,0.73], [0.01,0.84], [0.01,0.95]]
            # hd_ls  = [270,271,272,273,274] 
            # self.gen_images_from_pos(positions[self.current_step], hd_ls[self.current_step])



            if self.task == 'gen_data':
                file_name = (
                'matx' + f"{self.X_all[self.current_step]:.2f}" +
                '_y' + f"{self.Y_all[self.current_step]:.2f}" +
                '_HD' + f"{self.HD_all[self.current_step]:.2f}"
                )

                img_name = (
                'imgx' + f"{self.X_all[self.current_step]:.2f}" +
                '_y' + f"{self.Y_all[self.current_step]:.2f}" +
                '_HD' + f"{self.HD_all[self.current_step]:.2f}"
                )
                print('pos,hd', [self.X_all[self.current_step], self.Y_all[self.current_step]], self.HD_all[self.current_step] )
                if self.X_all[self.current_step] == 0.29 and self.Y_all[self.current_step] == 0.29 and self.HD_all[self.current_step] ==230:
                    print('reachedDebug')
                image_path = os.path.join('cells_kernels_images', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
                s_path =  os.path.join('cells_kernels', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
                img = self.gen_images_from_pos([float(self.X_all[self.current_step]), float(self.Y_all[self.current_step])], float(self.HD_all[self.current_step]))
                NeuralRate = self.gen_NeuralRate(img)
                np.save(os.path.join(s_path,file_name), NeuralRate)
                plt.imsave(os.path.join(image_path,img_name)+'.png', img)
                np.save(os.path.join(image_path,file_name)+'.npy', img)
                
            


            if self.task == 'implementation':
                
                img = self.gen_images_from_pos([self.current_position[0], self.current_position[1]], self.current_hd)
       
                NeuralRate =np.array(self.gen_NeuralRate(img))
                if self.init_flage:
                    self.init_flage = False
                    pass
                print('NR=', NeuralRate)
                if self.current_step == 100:
                    print('check here')
                self.current_position, self.current_hd = self.update_state(NeuralRate, self.current_position, self.current_hd)
                # self.current_position = self.current_position + np.array([0, 0.01])
                print(f'current_pos, current_hd = {self.current_position}, {self.current_hd}')
                self.postion_ls.append(self.current_position)
                self.hd_ls.append(self.current_hd)
                self.ratemap_ls.append(NeuralRate)
                self.image_ls.append(img)
                np.save('trj/postion_ls.npy', self.postion_ls)
                np.save('trj/hd_ls.npy', self.hd_ls)
                np.save('trj/u_ls.npy', self.u_ls)
                np.save('trj/image_ls.npy', self.image_ls)
                np.save('trj/ratemap_ls.npy', self.ratemap_ls)



            if self.task == 'lipt_test_line':
              
                print('pos,hd', [self.X_all[self.current_step], self.Y_all[self.current_step]], self.HD_all[self.current_step] )
                
                image_path = os.path.join('lipt_test_line', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
                s_path =  os.path.join('lipt_test_line', 'c'+str(self.cell_id), 'deg'+str(self.HD_all[self.current_step]))
                img = self.gen_images_from_pos([self.X_all[self.current_step], self.Y_all[self.current_step]], self.HD_all[self.current_step])
                NeuralRate = self.gen_NeuralRate(img)
                u = self.controller(NeuralRate, [self.X_all[self.current_step], self.Y_all[self.current_step]],  self.HD_all[self.current_step])
                # new_data = {"pos": np.array([self.X_all[self.current_step], self.Y_all[self.current_step]]), 'hd': self.HD_all[self.current_step], "img": img, "neural rate":NeuralRate, "u": u}
                # self.all_data.append(new_data)
                if not self.init_flage:
                    self.pos_ls.append(np.array([self.X_all[self.current_step], self.Y_all[self.current_step]]))
                    self.hd_ls.append(self.HD_all[self.current_step])
                    self.img_ls.append(img)
                    self.nr_ls.append(NeuralRate)
                    self.u_ls.append(u)


            if self.task == 'lipt_test_orientation':
            
                print('hd',  self.HD_all[self.current_step] )
                
                image_path = os.path.join('lipt_test_orientation', 'x'+str(self.pos[0])+ 'y'+str(self.pos[1]))
                s_path =  os.path.join('lipt_test_orientation','x'+str(self.pos[0])+ 'y'+str(self.pos[1]))
                img = self.gen_images_from_pos(self.pos, self.HD_all[self.current_step])
                NeuralRate = self.gen_NeuralRate(img)
                u = self.controller(NeuralRate, self.pos,  self.HD_all[self.current_step])
                # new_data = {"pos": np.array([self.X_all[self.current_step], self.Y_all[self.current_step]]), 'hd': self.HD_all[self.current_step], "img": img, "neural rate":NeuralRate, "u": u}
                # self.all_data.append(new_data)
                if not self.init_flage:
                    # self.hd_ls.append(self.HD_all[self.current_step])
                    self.img_ls.append(img)
                    self.nr_ls.append(NeuralRate)
                    self.u_ls.append(u)




          
            print(f'current step = {self.current_step} / {self.num_steps}')
            # Increment step
            if self.init_flage:
                self.init_flage =False
            else: 
                self.current_step += 1
            return Task.cont
          
        else:
            print('Completed')
            # Stop MATLAB engine and exit when done
            if self.task == 'lipt_test_line':
                
                save_dir =  os.path.join('lipt_test_line', 'c'+str(self.cell_id), 'deg'+str(self.hd))

                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                    print(f"Directory created: {save_dir}")
                else:
                    print(f"Directory already exists: {save_dir}")

                self.img_ls = np.array(self.img_ls)
                self.u_ls = np.array(self.u_ls)
                self.nr_ls = np.array(self.nr_ls)
                self.pos_ls = np.array(self.pos_ls)
                self.hd_ls = np.array(self.hd_ls)
                np.save(os.path.join(save_dir,'img_ls.npy'),self.img_ls)
                np.save(os.path.join(save_dir,'u_ls.npy'),self.u_ls)
                np.save(os.path.join(save_dir,'nr_ls.npy'),self.nr_ls)
                np.save(os.path.join(save_dir,'pos_ls.npy'),self.pos_ls)
                np.save(os.path.join(save_dir,'hd_ls.npy'),self.hd_ls)
                self.plt_lipt_line(save_dir)

            if self.task == 'lipt_test_orientation':
                name = f'x{self.pos[0]}_y{self.pos[1]}'
                if not os.path.isdir(os.path.join('lipt_test_orientation', name)):
                    os.mkdir(os.path.join('lipt_test_orientation', name))
                save_dir =  os.path.join('lipt_test_orientation', name)
                self.img_ls = np.array(self.img_ls)
                self.u_ls = np.array(self.u_ls)
                self.nr_ls = np.array(self.nr_ls)
                
               
                np.save(os.path.join(save_dir,'img_ls.npy'),self.img_ls)
                np.save(os.path.join(save_dir,'u_ls.npy'),self.u_ls)
                np.save(os.path.join(save_dir,'nr_ls.npy'),self.nr_ls)
                
                # np.save(os.path.join(save_dir,'hd_ls.npy'),self.hd_ls)
                self.plt_lipt_orin(os.path.join(save_dir, 'plt.png'))


                # with open(save_path, 'w') as json_file:
                #     json.dump(self.all_data, json_file, indent=5)
          
            self.eng.quit()
            sys.exit()

    
    def gen_images_from_pos(self, pos, hd):
        """
        Generates images for a given position and heading.
        Args:
            [x, y] positions.
             headings.

        Returns:
            generated image.
        """
        
    
        # nx = np.round(np.maximum(B, np.minimum((np.round(pos[0],2) - x_size / 2) * 1000 + N / 2, B + W - 1)), decimals=0)
        # ny = np.round(np.maximum(B, np.minimum((np.round(pos[1],2) - y_size / 2) * 1000 + N / 2, B + W - 1)), decimals=0)
        nx = np.maximum(B, np.minimum((pos[0] - x_size / 2) * 1000 + N / 2, B + W - 1))
        ny = np.maximum(B, np.minimum((pos[1] - y_size / 2) * 1000 + N / 2, B + W - 1))
        nhd = (np.round(float(hd),1) - 90) % 360
        # Set camera position and heading
        self.camera.setPos(nx, ny, Z)
        self.camera.setHpr(nhd, P, 0)
        

        # Capture the screenshot
        sr = self.win.getScreenshot()
        data = sr.getRamImage()
        image = np.frombuffer(data, np.uint8)
        image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
        image = np.flipud(image)

        # Normalize and resize the image
        img = image[:, :, 2].astype(np.float32) / 255.0
        img = cv2.resize(img, (150, 90), interpolation=cv2.INTER_AREA)

        # Apply Gaussian blur to reduce noise (optional)
        # img = cv2.GaussianBlur(img, (3, 3), 0)

        # Save the image for debugging
        plt.imsave(f'test/img/xyhd_{pos[0]}_{pos[1]}_{hd}.png', img, cmap='gray')

        return img
    
    def gen_NeuralRate(self, image):
        S = np.zeros([100,1])
        U = np.zeros([100,1])
        S_out, U_out = self.eng.generate_V1_RSC_model_response(image, S, U, nargout=2)
        return S_out


    def controller(self, neural_map, position, orinetation ):
        """
        Update position and head direction based on MATLAB outputs.
        """
        # if 0<position[0]<0.9 and 0.2<position[1]<0.3:
        #     u = np.array([[5],[-5]])
        # else:
        current_cell = self.findcell(position)
        
        K,Kb = self.control_gain_load.interpolate_contorlgains(current_cell, orinetation)
        # print('K=',K)
        # print('Kb=', Kb)

        u = K@neural_map+Kb
        print('u=',u)
        speed = 3
        # speed = 0.4
        u = u/np.linalg.norm(u)
        print('u_norm=',u)
    
        u = u*speed
    
        return u
    

    def update_state(self, neural_map, position, orinetation ,update_hd = 1):

        """
        Update position and head direction based on MATLAB outputs.
        """
        current_cell = self.findcell(position)
        u = self.controller(neural_map, position, orinetation)
        self.u_ls.append(u)
        if update_hd:
            new_hd  = (np.atan2(u[1], u[0])*180/np.pi)%360
            if np.abs(new_hd+360-orinetation)< np.abs(new_hd-orinetation):
                new_hd = new_hd+360
            eta = 0.8
            new_hd = ((1-eta)*orinetation+eta*float((new_hd)[0]))
            new_hd = np.sign(new_hd-orinetation)*min(np.abs(new_hd-orinetation), 18)+orinetation
            new_hd = new_hd%360
            # new_hd = new_hd//10*10
            print(new_hd)
            
        else:
            new_hd = orinetation
        print('new_hd=', new_hd )
       
       
        B_dis = np.eye(2)*self.dt
        ##heading angle in degree
       
       
        
        
        new_position = position+(B_dis@u).flatten()

        return new_position, new_hd
    

    def plt_lipt_line(self, save_dir):
        fig, ax = plt.subplots(2, figsize=(18, 16))
        for i in range(self.nr_ls.shape[1]):
            ax[0].plot(self.nr_ls[:,i])
        ax[0].set_title('Neural Rates')
        ax[1].set_title('Control')
        # ax[1].plot(self.pos_ls[:,0], self.pos_ls[:,1])
        ax[1].quiver(self.pos_ls[:,0], self.pos_ls[:,1], self.u_ls[:,0], self.u_ls[:,1])
        for i in range(len(self.cell_ls[self.cell_id].vrt)-1):
            ax[1].plot([self.cell_ls[self.cell_id].vrt[i,0], self.cell_ls[self.cell_id].vrt[i+1,0]], [self.cell_ls[self.cell_id].vrt[i,1], self.cell_ls[self.cell_id].vrt[i+1,1]], color = 'black')
        
        ax[1].plot([self.cell_ls[self.cell_id].vrt[0,0], self.cell_ls[self.cell_id].vrt[-1,0]], [self.cell_ls[self.cell_id].vrt[0,1], self.cell_ls[self.cell_id].vrt[-1,1]], color = 'black')
        for wall in (self.cell_ls[self.cell_id].bar):
            ax[1].plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax[1].plot([self.cell_ls[self.cell_id].exit_vrt[0][0], self.cell_ls[self.cell_id].exit_vrt[1][0]], [self.cell_ls[self.cell_id].exit_vrt[0][1], self.cell_ls[self.cell_id].exit_vrt[1][1]], color = 'green')
        plt.tight_layout()
        plt.savefig(save_dir+'/plt.png', dpi = 300)



    def plt_lipt_orin(self, save_dir):

        fig, ax = plt.subplots(4, figsize=(18, 16))
        for i in range(self.nr_ls.shape[1]):
            ax[0].plot(self.nr_ls[:,i])
        ax[0].set_title('Neural Rates')
        ax[1].set_title('Control_x')
        ax[1].plot(self.HD_all, self.u_ls[:,0])

        ax[1].set_title('Control_y')
        ax[2].plot(self.HD_all, self.u_ls[:,1])
        ax[3].plot(np.atan2(self.u_ls[:,1], self.u_ls[:,0]))
        ax[3].set_title('control Orientation')
        plt.tight_layout()
        plt.savefig(save_dir, dpi = 300)


# Main execution
if __name__ == '__main__':
    
    initial_position = np.array([0.4, 0.25])
    initial_hd = 0



    # initial_position = np.array([0.25, 0.35])
    # initial_hd = 270


    num_steps = 150

    task = 'implementation'
    app = MyApp(task)
    app.init_implementation(initial_position, initial_hd, num_steps)



    # task = 'lipt_test_line'
    # app = MyApp(task)
    # cell_id = 14
    # print(f'******************cell_id= {cell_id}')
    # app.init_lipt_test_line(cell_id= cell_id, hd=270)



    
    # task = 'lipt_test_orientation'
    # app = MyApp(task)
    # app.init_lipt_test_orientation(np.array([0.8,0.2]))



    # parser = argparse.ArgumentParser(description="Run MyApp with a specified cell ID.")
    # parser.add_argument("--cell_id", type=int, default=1, help="Specify the cell ID (default: 15)")
    
    # args = parser.parse_args()

    # task = 'gen_data'
    # app = MyApp(task)
    # # cell_id = 47
    # cell_id = args.cell_id 
    # print(f'******************cell_id= {cell_id}')
    # app.init_gen_data(cell_id= cell_id)

    app.run()
