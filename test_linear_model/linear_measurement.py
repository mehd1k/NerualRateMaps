import numpy as np
import matplotlib.pyplot as plt
import os
import sys
# Add parent directory to path to allow importing gen_controller
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gen_controller import cell, Control_cal, cell_ls, vectorize_matrix, map_address
def linear_measurement(x):
    '''linear measurement model'''
    C =  np.array([[1,2],[2,3],[3,4],[4,5]])
    D =  np.array([[1],[2],[3],[4]])
    # y = C@x+D
    # y = C@np.exp(x)+D
    # y =C@x**2+D
    # y = C@np.sin(x)+D
    y =D
    return y



A = np.zeros((2,2))
B = np.eye((2))
i_cell = 9  
directory_mat = None
directory_save =  '/home/mehdi/NerualRateMaps/test_linear_model'
print("###############################cell", str(i_cell))
cell_i = cell_ls[i_cell]
# s0=Control_cal(cell_i,A, B,ch =0.4*10**1, cv=22*10**-1, eps = 15*10**-3, sigma_max = 10**-6, dt = 0.001 , grid_size_x=10,
#                         grid_size_y=10, directory_mat =directory_mat , directory_save = directory_save )
s0=Control_cal(cell_i,A, B,ch =0.2*10**-2, cv=8*10**-4, eps = 1*10**-3, sigma_max = 10**-6, dt = 0.001 , grid_size_x=10,
                                          grid_size_y=10, directory_mat =directory_mat , directory_save = directory_save )

def generate_kernel(s0):
    '''generate kernel for the linear model'''
    num_points = 10
    size_measurement = 4
    xlist = np.linspace(s0.xmin, s0.xmax, num_points)
    ylist = np.linspace(s0.ymin, s0.ymax, num_points)
    ylist = ylist[::-1]
    output = np.zeros((size_measurement,num_points*num_points))
    print('xlist', xlist)
    print('ylist', ylist)
    for iy in range(num_points):
        for ix in range(num_points):
            x = xlist[ix]
            y = ylist[iy]
            pos = np.array([[x],[y]])
            measurement = linear_measurement(pos)
            output[:,map_address(ix,iy,num_points)] = measurement.flatten()
    return output

kernel = generate_kernel(s0)
pos = np.array([[0.15],[0.9]])
P = s0.obs.obs(pos)
print('P', P)
P_vec = vectorize_matrix(P)
measurement = linear_measurement(pos)
measurement2 = kernel@P_vec
print('measurement', measurement)
print('measurement2', measurement2)
s0.kernel_ls = [kernel]
s0.get_K()
s0.vector_F()
def get_vector_field(s0):
    '''get vector field for the linear model'''
    num_points = 30
    size_measurement = 4
    xlist = np.linspace(s0.xmin, s0.xmax, num_points)
    ylist = np.linspace(s0.ymin, s0.ymax, num_points)
    ux_ls = []
    uy_ls = []
    u2x_ls = []
    u2y_ls = []
    X,Y = np.meshgrid(xlist, ylist)
    X,Y = X.flatten(), Y.flatten()
    for i in range(len(X)):
        x = X[i]
        y = Y[i]
        pos = np.array([[x],[y]])
        measurement = linear_measurement(pos)
        u = (s0.K[0]@measurement).flatten()+s0.Kb.flatten()
        u = u/np.linalg.norm(u)
        P = s0.obs.obs(pos)
        P_vec = vectorize_matrix(P)
        measurement2 = s0.kernel_ls[0]@P_vec
        u2 = (s0.K[0]@measurement2)+s0.Kb
        u2 = u2/np.linalg.norm(u2)
        u2x_ls.append(u2[0].copy())
        u2y_ls.append(u2[1].copy())
        # print('u', u)
        # print('measurement', measurement)
        ux_ls.append(u[0].copy())
        uy_ls.append(u[1].copy())

    ax, fig = plt.subplots()
    fig.quiver(X,Y,ux_ls,uy_ls)
    # fig[1].quiver(X,Y,u2x_ls,u2y_ls)
    plt.show()
       
get_vector_field(s0)