"""
Created on Wed Aug  2 11:13:56 2023
    This file setup the arena. Each pixel represents 1 mm while the thickness
of the wall can be adjusted to generate arenas of different sizes.

@author: Yanbo Lian
"""


import numpy as np
import matplotlib.pyplot as plt
import math


MAZE_ID = "SouthWhite0.6_light0.6_120H50_Lshape"
light_scale = 0.6
N= 2049 # size of the whole arena with walls
W = 1200-1 # size of the arena; 1 pixel is 1mm
R_curtain = round(W*math.sqrt(2)/2 + 100)
Centre = [int((N-1)/2), int((N-1)/2)]
B = int((N-W)/2) # thickness of the wall
buffer = 20 # a buffer to avoid taking wierd images near the wall; 2 cm
wall_width = 30 # 3cm
B_real = B - buffer - (wall_width-1)
H = 500 # height of the wall: 50 cm
F = 200 # height of the floor: 20cm
C_H = 1500 # height of the curtain
card_size = 0.6 # percentage of the cue card relative to the wall
W_card = round(W*card_size)
if np.mod(W_card,2) == 1:
    W_card = W_card - 1


wall_color = 0.8 # color of the wall; 0 is black; small values are grey
floor_color = 0.5 # color of the floor; 0 is black; small values are grey
card_color = 1 # color of the cue card; 1 is white
curtain_color = 0 # color of the curtain

maze = np.zeros((N,N))
maze[B_real-1:B_real-1+wall_width,B_real-1:N-B_real+1] = (H+F)*np.ones((wall_width,N-2*B_real+2)) # north wall
maze[N-B_real-wall_width+1:N-B_real+1,B_real-1:N-B_real+1] = (H+F)*np.ones((wall_width,N-2*B_real+2)) # south wall
maze[B_real-1:N-B_real+1,B_real-1:B_real-1+wall_width] = (H+F)*np.ones((N-2*B_real+2,wall_width)) # west wall
maze[B_real-1:N-B_real+1,N-B_real-wall_width+1:N-B_real+1] = (H+F)*np.ones((N-2*B_real+2,wall_width)) # east wall
maze[B_real-1+wall_width:N+1-B_real-wall_width,B_real-1+wall_width:N+1-B_real-wall_width] = F*np.ones((W+2*buffer,W+2*buffer)) # floor

# L shape insersion
maze[B_real-1:Centre[0]-buffer,Centre[1]+buffer+1:Centre[1]+buffer+1+wall_width] = (H+F)*np.ones((Centre[0]-buffer-B_real+1,wall_width)) # vertical insertion
maze[Centre[0]-buffer-1:Centre[0]-buffer-1+wall_width,Centre[1]+buffer+1:N-B_real+1] = (H+F)*np.ones((wall_width,N-B_real-Centre[1]-buffer)) # horizontal insertion

plt.imsave(f'{MAZE_ID}.png',maze)
# plt.imsave(f'{MAZE_ID}.png',maze,origin='lower')

# Colormap of the arena
cmaze = np.zeros((N,N,3))
cmaze[B_real-1:B_real-1+wall_width,B_real-1:N-B_real+1,:] = wall_color*np.ones((wall_width,N-2*B_real+2,3)) # north wall
cmaze[N-B_real-wall_width+1:N-B_real+1,B_real-1:N-B_real+1,:] = wall_color*np.ones((wall_width,N-2*B_real+2,3)) # south wall
cmaze[B_real-1:N-B_real+1,B_real-1:B_real-1+wall_width,:] = wall_color*np.ones((N-2*B_real+2,wall_width,3)) # west wall
cmaze[B_real-1:N-B_real+1,N-B_real-wall_width+1:N-B_real+1,:] = wall_color*np.ones((N-2*B_real+2,wall_width,3)) # east wall
cmaze[N-B_real-wall_width+1:N-B_real+1,B_real-1+wall_width+int((W-W_card)/2):B_real-1+wall_width+int((W-W_card)/2)+W_card,:] = card_color*np.ones((wall_width,W_card,3)) # south wall
cmaze[B_real-1+wall_width:N+1-B_real-wall_width,B_real-1+wall_width:N+1-B_real-wall_width,:] = floor_color*np.ones((W+2*buffer,W+2*buffer,3)) # floor

# L shape insersion
cmaze[B_real-1:Centre[0]-buffer,Centre[1]+buffer+1:Centre[1]+buffer+1+wall_width,:] = wall_color*np.ones((Centre[0]-buffer-B_real+1,wall_width,3)) # vertical insertion
cmaze[Centre[0]-buffer-1:Centre[0]-buffer-1+wall_width,Centre[1]+buffer+1:N-B_real+1,:] = wall_color*np.ones((wall_width,N-B_real-Centre[1]-buffer,3)) # horizontal insertion

# plt.imsave(f'c{MAZE_ID}.png',cmaze)
plt.imsave(f'c{MAZE_ID}.png',cmaze,origin='lower')