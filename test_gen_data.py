
import numpy as np
import matplotlib.pyplot as plt
import math
# from shapely.geometry import Point, Polygon
from scipy.spatial import Delaunay
# import cvxpy as cp
import gurobipy as gp
from gurobipy import GRB
import sys
import scipy.io as sio
import os

from scipy.io import loadmat
from gen_controller import Control_cal, load_mat_files, observation, cell, cell_ls, vectorize_matrix
import json
with open('trj/vector_field_data.json', 'r') as f:
    vf_data = json.load(f)
deg = 270

A = np.zeros((2,2))
B = np.eye((2))

i_cell = 48
directory_mat = 'cells_kernels/c'+str(i_cell)+'/deg'
directory_save =  'cells_controllers/c'+str(i_cell)+'/deg'
print("###############################cell", str(i_cell))
cell_i = cell_ls[i_cell]
# s0=Control_cal(cell_i,A, B,ch =0.4*10**-2, cv=7*10**-3, eps = 12*10**-3, sigma_max = 10**-6, dt = 0.001 , grid_size_x=10,
#                         grid_size_y=10, directory_mat =directory_mat+str(deg) , directory_save = directory_save+str(deg) )


s0=Control_cal(cell_i,A, B,ch =0.4*10**-2, cv=7*10**-3, eps = 12*10**-3, sigma_max = 10**-6, dt = 0.001 , grid_size_x=10,
                        grid_size_y=10, directory_mat =directory_mat+str(deg) , directory_save = directory_save+str(deg) )   



# idx = 0
# data = vf_data[idx]                     
# pos = np.array([[data['position'][0]],[data['position'][1]]])
# s0.check_Probability_constraints(pos[0,0],pos[1,0])
x = 0.16
y = 0.79
nr = np.load(os.path.join(directory_mat+str(deg), f'nr_{x}_y{y}_HD{deg}.npy')).flatten()
pos = np.array([[x],[y]])
P = s0.obs.obs(pos)
P_vec = vectorize_matrix(P)
print('P', P)
# print('P_vec', P_vec)
ns_est = (s0.kernel_ls[0]@P_vec).flatten()
print('kernel_ls[0]@P_vec',ns_est)
# print('neural_rate', data['neural_rate'])
print('nr', nr)
print('difference', np.max(np.abs(ns_est - nr)))
print('difference between idx_i and np.argmax(P_vec)', np.max(np.abs(s0.idx_i - np.argmax(P_vec))))

print('difference between nr_i and nr', np.max(np.abs(s0.nr_i - nr)))
# json_file_ls = []
# npy_file_ls = []

# for filename in os.listdir(directory_mat+str(deg)):
#     if filename.endswith('.npy'):
#         npy_file_ls.append(filename)
#     if filename.endswith('.json'):
#         json_file_ls.append(filename)

# json_file_ls.sort()
# npy_file_ls.sort()

# # Test all file indices
# print(f'\nTesting {len(json_file_ls)} files:')
# all_match = True
# for file_index in range(len(json_file_ls)):
#     with open(os.path.join(directory_mat+str(deg), json_file_ls[file_index]), 'r') as f:
#         json_data = json.load(f)
#         x_pos = json_data['position']['x']
#         y_pos = json_data['position']['y']
#         neural_rate_json = np.array(json_data['neural_rate'])
#         npy_data = np.load(os.path.join(directory_mat+str(deg), npy_file_ls[file_index]))
    
#     pos = np.array([[x_pos],[y_pos]])
#     P = s0.obs.obs(pos)     
#     P_vec = P.reshape([-1,1])
    
#     # Try to get neural rate from exact position match first, otherwise use kernel
#     # Use tolerance-based matching since JSON positions may not exactly match file positions
#     found_match = False
#     tolerance = 0.01  # Allow small differences in position (JSON positions may have different precision)
    
#     if hasattr(s0, 'position_to_file_exact'):
#         # Find the file with position closest to the queried position
#         min_dist = float('inf')
#         best_match = None
        
#         for (fx, fy), file_data in s0.position_to_file_exact.items():
#             dist = np.sqrt((x_pos - fx)**2 + (y_pos - fy)**2)
#             if dist < tolerance and dist < min_dist:
#                 min_dist = dist
#                 best_match = file_data
        
#         if best_match is not None:
#             neural_rate1 = best_match.reshape(-1, 1)
#             found_match = True
    
#     if not found_match:
#         # Fall back to kernel lookup
#         neural_rate1 = s0.kernel@P_vec
    
#     # Check if they match
#     diff = np.abs(np.array(neural_rate1).flatten() - neural_rate_json)
#     max_diff = np.max(diff)
#     match = np.allclose(neural_rate1.flatten(), neural_rate_json, atol=1e-6)
    
#     if not match:
#         all_match = False
#         print(f'file_index {file_index:2d}: {json_file_ls[file_index]} - FAIL (max_diff: {max_diff:.6f})')
#         if file_index == 0:
#             print('neural_rate1', neural_rate1.flatten()[:10])
#             print('neural_rate_json', neural_rate_json[:10])
#             print('difference', diff[:10])

# if all_match:
#     print(f'\n✓ All {len(json_file_ls)} files match!')
# else:
#     print(f'\n✗ Some files do not match.')

