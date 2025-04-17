"""
Created on 20240606
@author: Yanbo Lian
"""
################### Change postion here ###########################
X = 0.6 # unit: m
Y = 0.3 # unit: m
HD = 90 # unit: degree
##################################################################
from south_white_L import B, N, F, W, H, MAZE_ID, light_scale # load the setup of environment
from direct.showbase.ShowBase import ShowBase
from direct.task import Task                      
from panda3d.core import GeoMipTerrain, loadPrcFileData, PointLight, AmbientLight, DirectionalLight
import pathlib
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat, savemat

Z = F+20 # camear is 2cm above the floor
P = 0 # 10 #-30 #0 #30  # math.atan2(H,(N-2*B)/10)*180/math.pi
pxmm = 1

v_peak = 130
yaw_sd = 340
cam_far = 3000 # see 0.8m far; 2000 for previous simulations

x_size = (W+1)/1000
y_size = (W+1)/1000
#Nsteps = 40000
VX, VY = 150, 90

TRAIL_PLOT = False #True  # False

map_file_path = 'C:\\Users\\mahdi\\Desktop\\yanbo_code'
map_img = map_file_path + os.sep + f"{MAZE_ID}.png"
map_img = pathlib.PurePosixPath(pathlib.Path(map_img))
cmap_img = map_file_path + os.sep + f"c{MAZE_ID}.png"
cmap_img = pathlib.PurePosixPath(pathlib.Path(cmap_img))

loadPrcFileData('', 'win-size {} {}'.format(VX, VY))
MAZE_OUT = "current_image"

# The current position and head direction
n = 2
y = Y * np.ones((n,))# unit: m
x = X * np.ones((n,))
hd = HD * np.ones((n,))

## Transform input trajectory into Panda3D reference
# (0,0) is the center of the terrain
# North is 0 degree and increase anti-clockwisely
nx = np.maximum(B,np.minimum((x-x_size/2)*1000+N/2,B+W-1))
ny = np.maximum(B,np.minimum((y-y_size/2)*1000+N/2,B+W-1))
nhd = (hd-90) % 360

Nsteps = n

class MyApp(ShowBase):                          # our 'class'
    def __init__(self):
        ShowBase.__init__(self)                        # initialise
                
     
        terrain = GeoMipTerrain("ratMaze")        # create a terrain
        terrain.setHeightfield(map_img)        # set the height map
        terrain.setColorMap(cmap_img)           # set the colour map
        terrain.setBruteforce(True)
        # terrain.setRoughness(1.0) # level of detail
        root = terrain.getRoot()          # maximum height
        root.reparentTo(self.render)                        # render from root
        root.setSz(H+F)
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
            # set limits
            plt.xlim(0, N)
            plt.ylim(0, N)
            
        alight = AmbientLight('alight')
        alight.setColor((light_scale, light_scale, light_scale, 1))
        alnp = self.render.attachNewNode(alight)
        alnp.setPos(N//2, N//2, 2000)
        self.render.setLight(alnp)
        
    def get_img(self, task):
        print(f"N: {task.frame}, hd: {hd[task.frame]:.1f}, nhd: {nhd[task.frame]:.1f}")
        self.camera.setPos(nx[task.frame], ny[task.frame], Z)
        self.camera.setHpr(nhd[task.frame], P, 0)
        self.pos.append(self.camera.getPos())
        self.hpr.append([hd[task.frame], P, 0])
        sr = self.win.getScreenshot()
        data = sr.getRamImage() # use data.get_data() instead of data in python 2
        image = np.frombuffer(data, np.uint8)
        image.shape = (sr.getYSize(), sr.getXSize(), sr.getNumComponents())
        image = np.flipud(image)
        print(image.shape)
            
        print(task.frame)
    
        next = image[:, :, 2]
        np.save('img.png', next)
        ShowBase.destroy(self)
       

if __name__ == '__main__':
    app = MyApp()                                   # our 'object'
    app.run() 
    app.get_img()                                      # away we go!