import os
import numpy as np
import matplotlib.pyplot as plt
from gen_controller import cell, cell_ls, cell_ls_vis
from matplotlib.patches import Polygon
import math
from gen_vec_field_from_perception import plot_vec
from find_controller_orientation import control_gain_load






def findcell(cell_ls, x):
        ls_flag=[]
        for i in range(len(cell_ls)):
            ls_flag.append(cell_ls[i].check_in_polygon(np.reshape(x,(1,2))))
        
        # print(ls_flag)
        return [i for i, x in enumerate(ls_flag) if x][0]
    

def create_triangle(x, y, angle, size=0.02):
    """
    Creates a triangle at (x, y) rotated by angle.
    
    :param x: x-coordinate of the triangle center
    :param y: y-coordinate of the triangle center
    :param angle: orientation of the triangle in radians
    :param size: size of the triangle
    :return: A matplotlib Polygon object
    """
    # Define triangle points relative to the center
    points = np.array([
        [size, 0],  # Tip of the triangle
        [-size / 2, -size / 3],  # Bottom left
        [-size / 2, size / 3]   # Bottom right
    ])
    # Rotate the triangle by the given angle
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    rotated_points = points @ rotation_matrix.T
    # Translate to the correct position
    translated_points = rotated_points + np.array([x, y])
    return Polygon(translated_points, closed=True, color='blue')


def visualization(bars):
        traj = np.load('trj/postion_ls.npy')
        hd_ls = np.load('trj/hd_ls.npy')*np.pi/180
        


        dx = np.cos(hd_ls)
        dy = np.sin(hd_ls)

        # Plot the trajectory
        fig, ax = plt.subplots()
        end_i = 1
        ax.plot(traj[:-end_i, 0], traj[:-end_i, 1], color='green', linestyle='dashed', linewidth=2, label="Trajectory")

        # Add orientation vectors using quiver
        ax.quiver(traj[:-end_i, 0], traj[:-end_i, 1], dx[:-end_i], dy[:-end_i], color='blue', scale=50, width=0.005, label="Orientation")
        
        



        # for i in range(len(traj)):
        #     triangle = create_triangle(traj[i, 0], traj[i, 1], hd_ls[i], size=0.01)
        #     ax.add_patch(triangle)
       
        
        ###Ploting cells
        for num_cell, cell in enumerate(cell_ls_vis):
            # if num_cell == 8:
            #      pass
            num_vrt = len(cell.vrt)-1
            for i in range(num_vrt):
                ax.plot([cell.vrt[i][0], cell.vrt[i+1][0]], [cell.vrt[i][1], cell.vrt[i+1][1]], color = 'gray')

            ax.plot([cell.vrt[0][0], cell.vrt[-1][0]], [cell.vrt[0][1], cell.vrt[-1][1]], color = 'gray')
            cx, cy = np.mean(cell.vrt, axis=0)
            ax.text(cx, cy, str(num_cell), ha='center', va='center', fontsize=7, color='black')
       
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        ax.set_aspect('equal')
        fig.savefig('trj/traj_pt.png', dpi= 300)




def gif_maker(bars):
        traj = np.load('trj/postion_ls.npy')
        hd_ls = np.load('trj/hd_ls.npy')*np.pi/180
        image_ls =np.load('trj/image_ls.npy')
        ratemap_ls = np.load('trj/ratemap_ls.npy')
        nstep = len(traj)
        for i_step in range(nstep):
             
            fig = plt.figure(figsize=(10, 10))

            # Add the larger subplot (first row)
            ax1 = plt.subplot2grid((5, 2), (0, 0), colspan=2, rowspan=2)   # Larger subplot spans across the first row
            ax1.set_title("Trajectory")
           



            dx = np.cos(hd_ls)
            dy = np.sin(hd_ls)

            # Plot the trajectory
           
            ax1.plot(traj[:i_step, 0], traj[:i_step, 1], color='green', linestyle='dashed', linewidth=2, label="Trajectory")

            # Add orientation vectors using quiver
            # ax1.quiver(traj[:i_step, 0], traj[:i_step, 1], dx[:i_step], dy[:i_step], color='blue', scale=50, width=0.005, label="Orientation")



            cgl = control_gain_load()
            cell_id = findcell(cell_ls, np.array([traj[i_step]]))
            # positions, control_inputs_x, control_inputs_y = cgl.interpolate_contorlvec(str(cell_id),hd_ls[i_step] )
            # control_inputs_x, control_inputs_y = control_inputs_x/(control_inputs_x**2+ control_inputs_y**2)**0.5, control_inputs_y/(control_inputs_x**2+ control_inputs_y**2)**0.5

            # ax1.quiver(positions[:, 0], positions[:, 1], control_inputs_x, control_inputs_y, angles='xy', scale_units='xy')



        
        
            
            ###Ploting cells
            for cell in cell_ls:
                num_vrt = len(cell.vrt)-1
                for i in range(num_vrt):
                    ax1.plot([cell.vrt[i][0], cell.vrt[i+1][0]], [cell.vrt[i][1], cell.vrt[i+1][1]], color = 'gray')

                ax1.plot([cell.vrt[0][0], cell.vrt[-1][0]], [cell.vrt[0][1], cell.vrt[-1][1]], color = 'gray')
        
            for env_vrt in bars:
                for i in range(len(env_vrt)-1):
                    ax1.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

                ax1.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
                # fig.show()



            ax1.set_aspect('equal')
            ax2 = plt.subplot2grid((5, 2), (2, 0))
            ax2.set_title("Image")
            ax2.imshow(image_ls[i_step])  # Example plot for the smaller subplot

            ax3 = plt.subplot2grid((5, 2), (2, 1)) # Right smaller subplot
            ax3.set_title("Neural Rate Map")
            im = ax3.imshow(ratemap_ls[i_step].reshape(10,10), cmap='viridis', vmin=0, vmax=1)  # Adjust vmin and vmax as needed

            # Add a colorbar
            cbar = plt.colorbar(im, ax=ax3)
            cbar.set_label('Intensity')  # Label for the colorbar
            

            ax4 = plt.subplot2grid((5, 2), (3, 0), colspan=2, rowspan=2)



            ax4.plot(traj[:i_step, 0], traj[:i_step, 1], color='green', linestyle='dashed', linewidth=2, label="Trajectory")

            # Add orientation vectors using quiver
            ax4.quiver(traj[:i_step, 0], traj[:i_step, 1], dx[:i_step], dy[:i_step], color='blue', scale=50, width=0.005, label="Orientation")



            

        
        
            
            ###Ploting cells
            for cell in cell_ls:
                num_vrt = len(cell.vrt)-1
                for i in range(num_vrt):
                    ax4.plot([cell.vrt[i][0], cell.vrt[i+1][0]], [cell.vrt[i][1], cell.vrt[i+1][1]], color = 'gray')

                ax4.plot([cell.vrt[0][0], cell.vrt[-1][0]], [cell.vrt[0][1], cell.vrt[-1][1]], color = 'gray')
        
            for env_vrt in bars:
                for i in range(len(env_vrt)-1):
                    ax4.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

                ax4.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
                # fig.show()



            ax4.set_aspect('equal')
            # cell_id = findcell(cell_ls,traj[i_step])



            plt.tight_layout()


            
            fig.savefig('trj/gif/i'+str(i_step)+'.png', dpi= 200)
            print(i_step)
            plt.close()

def _load_trj_array(path, allow_pickle=False):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return None
    try:
        return np.load(path, allow_pickle=allow_pickle)
    except (EOFError, ValueError, OSError):
        return None


def u_to_v_omega(u, hd, epsilon=0.1):
    """Same mapping as control_node_unicycle.offest_unicycle_model."""
    u = np.asarray(u).reshape(2)
    j_inv = np.array([
        [np.cos(hd), np.sin(hd)],
        [-np.sin(hd) / epsilon, np.cos(hd) / epsilon],
    ])
    v_omega = j_inv @ (u / 10.0)
    return np.clip(v_omega[0], -1, 1), np.clip(v_omega[1], -3, 3)


def v_omega_from_u_ls(u_ls, hd_ls, epsilon=0.1):
    hd_ls = np.asarray(hd_ls).reshape(-1)
    n = len(hd_ls)
    v_ls = np.zeros(n)
    omega_ls = np.zeros(n)
    for i in range(n):
        v_ls[i], omega_ls[i] = u_to_v_omega(u_ls[i], hd_ls[i], epsilon)
    return v_ls, omega_ls


def plot_data():
    postion_ls = np.load('trj/postion_ls.npy')
    hd_ls = np.load('trj/hd_ls.npy')
    odom_ls = np.squeeze(np.load('trj/odom_ls.npy'))
    u_ls = np.load('trj/u_ls.npy')
    v_ls = _load_trj_array('trj/v_ls.npy')
    omega_ls = _load_trj_array('trj/omega_ls.npy')
    if v_ls is None or omega_ls is None:
        print('trj/v_ls.npy or trj/omega_ls.npy missing or empty; deriving from u_ls and hd_ls')
        v_ls, omega_ls = v_omega_from_u_ls(u_ls, hd_ls)
    else:
        v_ls = np.asarray(v_ls).reshape(-1)
        omega_ls = np.asarray(omega_ls).reshape(-1)
    vx = v_ls * np.cos(hd_ls[1:])
    vy = v_ls * np.sin(hd_ls[1:])
    fig, ax = plt.subplots(3)
    ax[0].plot(vx, label ='cmd')
    ax[0].plot(odom_ls[:, 0], label ='odom')
    ax[0].legend()
    ax[1].plot(vy, label ='cmd')
    ax[1].plot(odom_ls[:, 1], label ='odom')
    ax[1].legend()
    ax[2].plot(omega_ls, label ='cmd')
    ax[2].plot(odom_ls[:, 2], label ='odom')
    ax[2].legend()
    os.makedirs('trj/data', exist_ok=True)
    plt.tight_layout()
    plt.savefig('trj/data/plot_odom.png', dpi= 200)
    plt.show()
    
   
    # for i in range(len(postion_ls)):
    #     plt.figure()


bars = [[[0, 1.2],[0, 0], [1.2, 0], [1.2 ,0.6], [0.6, 0.6], [0.6, 1.2]] ]


visualization(bars)
# plot_data()
# gif_maker(bars)       