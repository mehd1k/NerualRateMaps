
import os
import re
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from gen_controller import cell, cell_ls





def plot_vec(cell_id, cell_ls,hd, plot=True):
    
    


    dir_controller = 'cells_controllers/c'+str(cell_id)+'/deg'+str(hd) 
    # Directory containing the MATLAB files
    directory = 'cells_kernels/c'+str(cell_id)+'/deg'+str(hd)  

    # Define the controller parameter k
    K = np.load(os.path.join(dir_controller,'K.npy'))  # Adjust this based on your controller
    
    Kb = np.load(os.path.join(dir_controller,'Kb.npy'))


    # Lists to hold the positions and control inputs
    positions = []
    control_inputs_x = []
    control_inputs_y = []
    from gen_controller import cell, Control_cal
    # Regular expression to extract the x and y values from filenames
    filename_pattern = r'matx([-+]?\d*\.\d+|\d+)_y([-+]?\d*\.\d+|\d+)_.*\.npy'

    # Loop over all files in the directory
    for filename in os.listdir(directory):
        if filename.endswith('.npy'):
            # Extract x and y positions from the filename
            match = re.search(filename_pattern, filename)
            if match:
               
                x_pos = float(match.group(1))
                y_pos = float(match.group(2))
                if x_pos ==0.01 and y_pos ==0.31:
                    print('reached')

                # Load the MATLAB file
                file_path = os.path.join(directory, filename)
                # data = scipy.io.loadmat(file_path)

                # Extract S_new
                # S_new = data['S_new']
                # S_new = np.array(S_new)
                S_new = np.load(file_path)
                # Assuming S_new is a 2D vector, calculate control input
                u = K@S_new+Kb
            

                # Store the position and control input
                positions.append([x_pos, y_pos])
                control_inputs_x.append(u[0])
                control_inputs_y.append(u[1])

    # Convert lists to numpy arrays for easier manipulation
    positions = np.array(positions)
    control_inputs_x = np.array(control_inputs_x)
    control_inputs_y = np.array(control_inputs_y)

    # Create the vector field
    magnitudes = np.sqrt(control_inputs_x**2 + control_inputs_y**2)
    max_magnitude = np.max(magnitudes)

    # Set the scale based on the maximum magnitude
    scale_factor = max_magnitude * 50 # You can adjust this factor to tune the scaling
    if plot == False:
        return positions, control_inputs_x, control_inputs_y

    if plot:
        fig, ax = plt.subplots()        
        for i in range(len(cell_ls[cell_id].vrt)-1):
            ax.plot([cell_ls[cell_id].vrt[i,0], cell_ls[cell_id].vrt[i+1,0]], [cell_ls[cell_id].vrt[i,1], cell_ls[cell_id].vrt[i+1,1]], color = 'black')

        ax.plot([cell_ls[cell_id].vrt[0,0], cell_ls[cell_id].vrt[-1,0]], [cell_ls[cell_id].vrt[0,1], cell_ls[cell_id].vrt[-1,1]], color = 'black')
        for wall in (cell_ls[cell_id].bar):
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax.plot([cell_ls[cell_id].exit_vrt[0][0], cell_ls[cell_id].exit_vrt[1][0]], [cell_ls[cell_id].exit_vrt[0][1], cell_ls[cell_id].exit_vrt[1][1]], color = 'green')
        # Create the vector field plot with scaled arrows
        # ax.figure(figsize=(10, 7))
        ax.quiver(positions[:, 0], positions[:, 1], control_inputs_x, control_inputs_y, angles='xy', scale_units='xy', scale=scale_factor)
        # ax.xlabel('X')
        # ax.ylabel('Y')
        # ax.grid(True)

        plt.savefig(os.path.join(dir_controller,'vec_field_with_realperception'))
        plt.close()


def get_control(cell_ls,cell_id, hd, x_pos, y_pos):
    '''given the x postion it calculates controller and loads neural maps'''








    dir_controller = 'cells_controllers/c'+str(cell_id)+'/deg'+str(hd) 
    # Directory containing the MATLAB files
    directory = 'cells_kernels/c'+str(cell_id)+'/deg'+str(hd) 

    # Define the controller parameter k
    K = np.load(os.path.join(dir_controller,'K.npy'))  # Adjust this based on your controller
    
    Kb = np.load(os.path.join(dir_controller,'Kb.npy'))


    # Lists to hold the positions and control inputs
    positions = []
    control_inputs_x = []
    control_inputs_y = []
    from gen_controller import cell, Control_cal
    # Regular expression to extract the x and y values from filenames
    filename  =  'matx'+str(x_pos)+'_y'+str(y_pos)+'_HD'+str(hd)+'.00.npy'
    # filename_pattern = r'matx([-+]?\d*\.\d+|\d+)_y([-+]?\d*\.\d+|\d+)_.*\.mat'
    file_path = os.path.join(directory, filename)
    S_new = np.load(file_path)

    # Extract S_new
    
    print(f'neural rate: {S_new.flatten()}')

    u = K@S_new+Kb
    print('control',u)
    print('control_norm',u/np.linalg.norm(u))
    # Loop over all files in the directory





# for hd in range(0,360,10):
#     cell_id = 2
#     print('hd',str(hd))
#     plot_vec(cell_id,cell_ls, str(hd)) 
if __name__ == '__main__':
    get_control(cell_ls,cell_id=14,hd=270,x_pos='0.16',y_pos='0.29')