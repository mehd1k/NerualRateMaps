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
       
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        ax.set_aspect('equal')
        fig.savefig('trj/traj_pt.png', dpi= 600)




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

    


bars = [[[0, 1.2],[0, 0], [1.2, 0], [1.2 ,0.6], [0.6, 0.6], [0.6, 1.2]] ]


visualization(bars)
# gif_maker(bars)       