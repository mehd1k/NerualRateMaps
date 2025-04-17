

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

# def load_rate_map():
#     nx_csv = 41
#     ny_csv = 41
#     rate_maps_csv  = np.loadtxt('rate_maps.csv', delimiter=',')
#     rate_maps = np.zeros([100,nx_csv,ny_csv])


#     # rate_maps = np.zeros_like(rate_maps_csv)
#     # ly = rate_maps_csv.shape[1]
#     # for i in range(ly-1):
#     #     rate_maps[:,(i+1)] = rate_maps_csv[:,ly-(i+1)]

#     # rate_maps = np.rot90(rate_maps, 3)
#     # rate_maps = np.flipud(rate_maps)
   
#     for i in range(100):
#         rate_maps[i,:,:] = np.flip(np.reshape(rate_maps_csv[:,i], [nx_csv, ny_csv]),0)
#         # rate_maps_sum += rate_maps[i,:,:]
#     return rate_maps
def load_mat_files(directory,mod = 1):
    '''Get the list of all .mat files in the directory'''
    # if mod == 0:
    #     num_neurons = 100
    #     num_kernels = 100
    #     output = np.zeros([num_neurons,num_kernels])
    #     mat_files = [f for f in os.listdir(directory) if f.endswith('.npy')]
        
    #     # # Sort the files to ensure consistent order
    #     # mat_files.sort()

    #     # # Load the first 100 .mat files or fewer if there are less than 100
    #     # loaded_data = []
    #     for i, mat_file in enumerate(mat_files[:100]):
    #         file_path = os.path.join(directory, mat_file)
       
    #         output[:,i] = np.load(file_path)
    #         # output[:,i] = loadmat(file_path)['S_new'].reshape([num_neurons])
        
    #     # if len(loaded_data) != 100:
    #     #     loaded_data.append(loadmat(file_path)['S_new'].flatten())
    #     # return np.array(loaded_data).T
    #     return output
    
    
    
    
    if mod == 1:

        grid_data = {}

        # Iterate through the directory and process each .mat file
        for filename in os.listdir(directory):
            if filename.endswith('.npy'):
                # Extract x and y coordinates from the filename
                # Assumes format like 'matx0.01_y0.89_HD0.00.mat'
                name_parts = filename.split('_')
                x_str = name_parts[0].replace('matx', '')
                y_str = name_parts[1].replace('y', '')
                
                x = float(x_str)
                y = float(y_str)
                
                # Load the .mat file
                # mat_data = sio.loadmat(os.path.join(directory, filename))
                
                # the measurement vector 
                measurement = np.load(os.path.join(directory, filename)).flatten()  # Replace 'measurement' with the actual variable name
                
                # Store the data in a dictionary with keys as (x, y) tuples
                grid_data[(x, y)] = measurement

        # Now grid_data contains your measurements organized by (x, y) positions.
        # You can further organize this into a structured 2D or 3D array if needed.

        # Optionally, convert the grid data to a 2D or 3D numpy array for easier processing
        # Example: storing in a 2D numpy array (grid)
        x_coords = sorted(set([coord[0] for coord in grid_data.keys()]))
        y_coords = sorted(set([coord[1] for coord in grid_data.keys()]))
        # y_coords = sorted(set([coord[1] for coord in grid_data.keys()]))[::-1]

        # Initialize an empty grid (assuming all measurements are vectors of size 100)
        grid_shape = (len(x_coords), len(y_coords), 100)  # Adjust this if needed
        grid_array = np.empty(grid_shape)
        output = np.zeros([100,100])
        ix = 0
        # Fill the grid
        for j, y in enumerate(y_coords):
            for i, x in enumerate(x_coords):
           
                grid_array[i, j, :] = grid_data[(x, y)]
                output[:,ix ] =grid_data[(x, y)]
                ix +=1
        output =np.array(output)
 
        return output

    

def rate_maps_select(i_cell, rate_maps):
    ##########retruns the cell rate maps for the given cell_number
    if i_cell == 0:
        return rate_maps[:,1:10, 1:10]
    if i_cell == 1:
        return rate_maps[:,10:20, 1:10]
    elif i_cell == 2:
        return rate_maps[:,20:30,1:10]
    elif i_cell == 3:
        return rate_maps[:,30:40,1:10]
    elif i_cell == 4:
        return rate_maps[:,30:40,10:20]
    elif i_cell == 5:
        return rate_maps[:,30:39,20:30]
    elif i_cell == 6:
        return rate_maps[:,30:39,30:40]
    elif i_cell == 7:
        return rate_maps[:,20:30,30:40]
    elif i_cell == 8:
        return rate_maps[:,20:30,20:30]
    elif i_cell == 9:
        return rate_maps[:,20:30,10:20]
    elif i_cell == 10:
        return rate_maps[:,10:20,10:20]
    elif i_cell == 11:
        return rate_maps[:,1:10,10:20]




def rate_maps_select_triangles(i_cell, rate_maps):
    ##########retruns the cell rate maps for the given cell_number
    if i_cell == 0:
        return rate_maps[:,1:10, 1:20]
    if i_cell == 1:
        return rate_maps[:,1:20, 1:10]
    elif i_cell == 2:
        return rate_maps[:,10:30,1:10]
    elif i_cell == 3:
        return rate_maps[:,20:40,1:10]
    elif i_cell == 4:
        return rate_maps[:,30:40,0:20]
    elif i_cell == 5:
        return rate_maps[:,30:40,10:30]
    elif i_cell == 6:
        return rate_maps[:,30:39,20:39]
    elif i_cell == 7:
        return rate_maps[:,21:40,30:39]
    elif i_cell == 8:
        return rate_maps[:,20:30,20:40]
    elif i_cell == 9:
        return rate_maps[:,20:30,10:30]
    elif i_cell == 10:
        return rate_maps[:,10:30,10:20]
    elif i_cell == 11:
        return rate_maps[:,1:20,10:20]


class observation():
    def __init__(self, landmark_position, sigma_max, epsilon, grid_size, dimensions):
        self.xl = landmark_position
        self.sm = sigma_max
        self.eps = epsilon
        
        
        
        self.dy = dimensions[1]
        self.dx = dimensions[0]
        self.nx = grid_size[0]
        self.ny = grid_size[1]
        
    def obs(self, x_robot):
        # self.Po = np.zeros(self.ny,self.nx)
        Po = np.zeros((self.ny,self.nx))
        dis_rel = -self.xl+x_robot
        
        ix = int(np.round ( (dis_rel[0])/(self.dx)*(self.nx-1)))
        iy =  int( np.round ((dis_rel[1])/(self.dy)*(self.ny-1)))
       
        Po[iy,ix] = 1
    
        return Po
    def guass(self, x):
        Po = np.zeros((self.ny,self.nx))
        dis_rel = self.xl-x
        mu_x = dis_rel[0][0]
        mu_y =  dis_rel[1][0]
        flag_sign = np.random.randint(1,2)
        mean = np.array([mu_x,mu_y])
        if flag_sign == 1:
            mean = mean + self.eps
        else:
            mean = mean- self.eps
    
        # mean = mean[0]
        for iy in range(self.ny):
            for ix in range(self.ny):
                x_g = ix*2*self.dx/(self.nx-1)-self.dx
                y_g = self.dy -iy*2*self.dy/(self.ny-1)
                Po[iy,ix] = np.exp(-1/(2*self.sm)*( (x_g-mean[0])**2+(y_g-mean[1])**2))

        Po = Po/np.sum(Po)
        
        # lims = (-self.dx, self.dx) # support of the PDF
        # xx, yy = np.meshgrid(np.linspace(*lims, self.nx), np.linspace(*lims, self.nx))
        # points = np.stack((xx, yy), axis=-1)
        # flag_sign = np.random.randint(1,2)
        
        # if flag_sign == 1:
        #     # mean = np.array([iy,ix]) + self.eps*np.random.random((1,2))*self.nx/self.dx
        # else:
        #     mean = np.array([iy,ix])- self.eps*np.random.random((1,2))*self.nx/self.dx
        
        # plt.imshow(pdf)
        # plt.show()
        return Po
    def just_mean(self, x):
        return self.xl-x

       

class Discrertized_Linear_Controller():
    def __init__(self, A, B, dt):
        self.A = A
        self.B = B
        self.dt = dt
        self.A_dis = np.zeros_like(A)
        self.B_dis = np.zeros_like(B)
    def A_discretized_calculator(self):
        A_dis = np.eye(len(self.A))
        A_n = A_dis
        for n in range(20):
            A_n = self.A@A_n
            new_trem = 1/math.factorial(n)*A_n*self.dt**n
            A_dis = A_dis + new_trem
        self.A_dis = A_dis
    
    
    def B_discretized_calculator(self):
        B_dis = self.B*self.dt
        B_n = B_dis
        for n in range(2,20):
            B_n = self.A@B_n
            new_trem = 1/math.factorial(n)*B_n*self.dt**n
            B_dis = B_dis + new_trem
        self.B_dis = B_dis

    def __call__(self) :
        self.A_discretized_calculator()
        self.B_discretized_calculator()
        return self.A_dis, self.B_dis



def vectorize_matrix(P):
    ny = np.shape(P)[0]
    nx = np.shape(P)[1]
    out = np.zeros((int(nx*ny), 1))
    for iy in range(ny):
        for ix in range(nx):
            out[int(iy*nx+ix)] = P[iy, ix]
   
    return out



def cbf(x_pos, AH, bH):
     c =(AH@x_pos+np.reshape(bH, (-1,1) ))
     if all(c>=0):
         return 1
     else:
         return 0

def polygon(x_pos, Ax, bx):
    c = Ax@x_pos+np.reshape(bx, (-1,1) )
    if all(c<=0):
        return 1
    else:
        return 0







class Control_cal():
    def __init__(self, cell, A, B, dt,ch,cv, eps, sigma_max , grid_size_x, grid_size_y , directory_mat, directory_save ,con_lim = 4):
        
        self.nx = grid_size_x
        self.ny = grid_size_y
        self.gs = [self.nx,self.ny]
       
        self.cell = cell
      
       
        self.ch = ch
        self.cv = cv
        self.o = np.reshape(cell.exit_vrt[1],(2,1))
        # self.rate_maps_cell = rate_maps_cell
        
        
        self.vrt = np.array(cell.vrt)
        self.xmin = np.min(self.vrt[:,0])
        self.ymin = np.min(self.vrt[:,1])
        self.xmax = np.max(self.vrt[:,0])
        self.ymax = np.max(self.vrt[:,1])
        
        dx = (self.xmax-self.xmin)
        dy = (self.ymax-self.ymin)
        dmax = max(dx, dy)
        
        self.d = [dx,dy]
        self.Po_ls =[]
        self.center = np.mean(self.vrt, axis=0)
        self.eps = eps
        self.sigma_max = sigma_max
        # self.eps = max(2*dx/nx,2*dy/ny)*1.1
        # self.eps = 10
        # self.sigma_max = self.eps**2
        # self.sigma_max = 160
        self.A = A
        self.B = B
        self.dt = dt
    
        self.l = np.array([[self.xmin], [self.ymin]])
        self.Ax, self.bx = self.get_Axbx()
        
        self.tx, self.ty = self.get_theta()
        
        self.AH,  self.bH = self.get_Ahbh()

        self.obs = observation(self.l, self.eps, self.sigma_max, self.gs, self.d)

        self.v = self.get_v()
        self.grid = []
        self.con_lim = con_lim
        self.directory_save =directory_save
  
        self.kernel  = load_mat_files(directory_mat)
        
        
        
        # n =100
        # A = np.random.randn(100, 100)
        # # Perform QR decomposition
        # Q, R = np.linalg.qr(A)
        # self.kernel = Q
    
     
   

    
    def get_theta(self):
       
        # nx,ny = self.nx, self.ny
        # nx,ny = self.gs[0],self.gs[1]

        nx, ny = 10,10
        dx= self.d[0]
        dy= self.d[1]
    
        
        self.tx = np.zeros((ny,nx))
        self.ty = np.zeros((ny,nx))
        for ix in range(nx):
            
                self.ty[:, ix] = np.linspace(self.ymin,self.ymax,ny)

      
        for iy in range(ny):
            self.tx[iy, :] = np.linspace(self.xmin,self.xmax,nx)
               

        txT = vectorize_matrix(self.tx)
        txT = np.reshape(txT, [1,-1])

        tyT = vectorize_matrix(self.ty)
        tyT = np.reshape(tyT, [1,-1])
        self.txT = txT
        self.tyT = tyT
        self.U = np.array([self.txT[0],self.tyT[0]])
        
        szp = int(nx*ny)
        ### Making Ap
        self.Ap = np.zeros((4,szp))
        
        
        self.Ap[0:2] = self.U
        self.Ap[2:] = -self.U
        ### Making bp
        self.bp = np.array([ [-self.eps], [-self.eps], [-self.eps], [-self.eps]] )
        self.Ax2 = np.array([ [-1,0],[0,-1], [1,0], [0,1]])
        return self.tx,self.ty

    def check_Probability_constraints(self,x_pos, y_pos):
        pos = np.array([[x_pos],[y_pos]])
        P = self.obs.obs(pos)     
        P_vec = P.reshape([-1,1])
        x_obs = self.U@P_vec
        print('U@P=',x_obs)
        print(self.Ap@P_vec+self.Ax2@pos+self.bp)

    
    def plot_cell(self):
        fig, ax = plt.subplots()
        for i in range(len(self.vrt)-1):
            ax.plot([self.vrt[i,0], self.vrt[i+1,0]], [self.vrt[i,1], self.vrt[i+1,1]], color = 'black')
        
        ax.plot([self.vrt[0,0], self.vrt[-1,0]], [self.vrt[0,1], self.vrt[-1,1]], color = 'black')
        for wall in (self.cell.bar):
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax.plot([self.cell.exit_vrt[0][0], self.cell.exit_vrt[1][0]], [self.cell.exit_vrt[0][1], self.cell.exit_vrt[1][1]], color = 'green')
    def get_Axbx(self):
        Ax = []
        bx = [] 
      
        for iv in range(len(self.vrt)-1):
            Ai = np.array([-self.vrt[iv+1][1]+self.vrt[iv][1],  self.vrt[iv+1][0]-self.vrt[iv][0]])
            bi = self.vrt[iv][0]*self.vrt[iv+1][1] - self.vrt[iv][1]*self.vrt[iv+1][0]
            
            # check1 = Ai@np.array(self.vrt[iv+1])+bi
            # check2 = Ai@np.array(self.vrt[iv])+bi
            
            if Ai@self.center+bi > 0 :
                Ai = -Ai
                bi = -bi
            Ax.append(Ai)
            bx.append(bi)
        
        
        ###last and first vertices
        Ai = np.array([-self.vrt[-1][1]+self.vrt[0][1],  self.vrt[-1][0]-self.vrt[0][0]])
        bi = self.vrt[0][0]*self.vrt[-1][1] - self.vrt[0][1]*self.vrt[-1][0]
        
        # check1 = Ai@np.array(self.vrt[iv+1])+bi
        # check2 = Ai@np.array(self.vrt[iv])+bi
        
        if Ai@self.center+bi > 0 :
            Ai = -Ai
            bi = -bi
        Ax.append(Ai)
        bx.append(bi)
       
        return np.array(Ax),np.reshape(np.array(bx),(-1,1))
    

    def get_Ahbh(self):
        Ah = []
        bH = [] 
        for seg in self.cell.bar: 
            x1,y1,x2,y2 = seg[0][0],seg[0][1],seg[1][0],seg[1][1]
            Ai = np.array([y1-y2, -(x1-x2)])
            ln = np.linalg.norm(Ai)
            Ai = Ai/ln
            bi = y1*(x1-x2)-x1*(y1-y2)
            bi = bi/ln
            # check1 = Ai@np.array(self.vrt[iv+1])+bi
            # check2 = Ai@np.array(self.vrt[iv])+bi
            
            if Ai@self.center+bi< 0 :
                Ai = -Ai
                bi = -bi
            Ah.append(Ai)
            bH.append(bi)

                
       
  
       
        return np.array(Ah),np.reshape(np.array(bH),(-1,1))
    
    def u(self, P_vec):
        u = self.K@self.kernel@P_vec+self.Kb
        # con_lim = self.con_lim
        # if np.abs(u[0])>con_lim:
        #     u[0] = con_lim*u[0]/np.abs(u[0])
        # if np.abs(u[1])>con_lim:
        #     u[1] = con_lim*u[1]/np.abs(u[1])
        return u
    


    def u_postion(self, x_pos, y_pos):
        
        
        pos = np.array([[x_pos],[y_pos]])
        P = self.obs.obs(pos)     
        P_vec = P.reshape([-1,1])
        neural_rate = self.kernel@P_vec
        u = self.K@neural_rate+self.Kb
        # con_lim = self.con_lim
        # if np.abs(u[0])>con_lim:
        #     u[0] = con_lim*u[0]/np.abs(u[0])
        # if np.abs(u[1])>con_lim:
        #     u[1] = con_lim*u[1]/np.abs(u[1])
        print('neural rate', neural_rate)
        print('control', u)
        return u
    


    def check_P_U(self, x_pos, y_pos):
        
        
        pos = np.array([[x_pos],[y_pos]])
        P = self.obs.obs(pos)     
        P_vec = P.reshape([-1,1])
        x_obs = self.U@P_vec
        print(x_obs)
       
    
    
   
    def vector_F(self):

        obs = observation(self.l, self.eps, self.sigma_max, self.gs, self.d)
        CD = Discrertized_Linear_Controller(self.A, self.B, self.dt)
        A_dis, B_dis = CD()
        X  = np.linspace(self.xmin,self.xmax,10)
        Y = np.linspace(self.ymin,self.ymax,10)
        ux_ls = []
        uy_ls = []
        
        xg =[]
        yg =[]
        # self.cpox = np.zeros((len(Y),len(X)))
        # self.cpoy = np.zeros((len(Y),len(X)))
        for iy in range(len(Y)):
            for ix in range(len(X)):
           
                x =X[ix]
                y = Y[iy]
                x_old = np.array([[x],[y]])
                # if all(self.Ax@x_old+self.bx <0):
            
                Po = vectorize_matrix(obs.obs(x_old))
                # Po = obs.obs(x_old).T.flatten().reshape([-1,1])
                u =self.u(Po)
                uc = np.copy(u)/np.linalg.norm(u)
            
                
                ux_ls.append(uc[0]) 
                uy_ls.append(uc[1])
                xg.append(x)
                yg.append(y)
            
        fig, ax = plt.subplots()        
        for i in range(len(self.vrt)-1):
            ax.plot([self.vrt[i,0], self.vrt[i+1,0]], [self.vrt[i,1], self.vrt[i+1,1]], color = 'black')
        
        ax.plot([self.vrt[0,0], self.vrt[-1,0]], [self.vrt[0,1], self.vrt[-1,1]], color = 'black')
        for wall in (self.cell.bar):
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], color = 'red')
        ax.plot([self.cell.exit_vrt[0][0], self.cell.exit_vrt[1][0]], [self.cell.exit_vrt[0][1], self.cell.exit_vrt[1][1]], color = 'green')
        sc =1
        ux_ls = sc*np.array(ux_ls)
        uy_ls = sc* np.array(uy_ls)
        ax.quiver(xg,yg,ux_ls,uy_ls,angles='xy', scale_units='xy')
        # print('u_min', np.min(np.abs(ux_ls)), np.min(np.abs(uy_ls)))
        # np.save('ux_ls.npy',ux_ls)
        # np.save('uy_ls.npy',uy_ls)
        fig.show()
        # fname = 'vec_field_mod'+str(i_cell)+'ch_'+str(self.ch)+'cv_'+str(self.cv)+'es_'+str(self.eps)+'_'+str(self.sigma_max)+'.png'
        fname = os.path.join(self.directory_save, 'vec_field.png')
        plt.show()
        fig.savefig(fname, dpi = 300)
    
    def check_AxAH(self):
        ny = self.nx
        nx = self.ny
        Cx = np.zeros((ny,nx))
        Ccbf = np.zeros((ny,nx))
        
        X = np.linspace(np.min(self.vrt[:,0])*0.9,np.max(self.vrt[:,0])*1.1,nx)
        Y = np.linspace(np.min(self.vrt[:,1])*0.9,np.max(self.vrt[:,1])*1.1, ny)
        for iy in range(len(Y)):
            for ix in range((len(X))):
                x_pos = np.array([ [X[ix]], [Y[iy]]] )
                # if all(Ax@x_pos+bx<=0):
                Cx[iy,ix] = polygon(x_pos,self.Ax,self.bx)
                Ccbf[iy,ix] = np.reshape(cbf(x_pos, self.AH, self.bH),[1,-1])
                # else:
                #     Cx[iy,ix] = polygon(x_pos,Ax,bx)
                #     Ccbf[:,iy,ix] = np.nan*np.ones_like(Ccbf[:,iy,ix])
                
        Ccbf = np.flip(Ccbf, 0)
        Cx = np.flip(Cx, 0)
        print('*****************CBF**************************')
        print(Ccbf)
        
                
        print('*****************Cx**************************')
        print(Cx)
        
        
    def get_v(self):
        ###normal vector to the exit face
        t_exit = self.cell.exit_vrt[0]-self.cell.exit_vrt[1]
        n_exit = np.array([-t_exit[1], t_exit[0]])
        V = n_exit @(self.center- self.cell.exit_vrt[0])
        if V < 0 :
            n_exit = -n_exit
        return np.reshape(np.array(n_exit)/np.linalg.norm(n_exit), (1,2))
    
    
    def check_in_polygon(self, p):
        """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
        p = np.reshape(p,(1,2))
        from scipy.spatial import Delaunay
        if not isinstance(self.vrt,Delaunay):
            hull = Delaunay(self.vrt)

        return (hull.find_simplex(p)>=0)[0]
    def gauss_kernel(self, mean,var):
        nx,ny = self.nx, self.ny
        dx= self.d[0]
        dy= self.d[1]
    
        Po = np.zeros((ny,nx))
        
        for iy in range(ny):
            for ix in range(nx):
                x_g = ix*2*self.d[0]/(nx-1)-self.d[0]
                y_g = self.d[1] -iy*2*self.d[1]/(ny-1)
                Po[iy,ix] = np.exp(-1/(2*var)*( (x_g-mean[0])**2+(y_g-mean[1])**2))

        Po = Po/np.sum(Po)
        self.Po_ls.append(Po)
        
        plt.imshow(Po)
        plt.show()
        
        tx = np.zeros((ny,nx))
        ty = np.zeros((ny,nx))
        for ix in range(nx):
            
                ty[:, ix] = Po[:,ix]

      
        for iy in range(ny):
            tx[iy, :] = Po[iy,:]
               

        txT = vectorize_matrix(tx)
        txT = np.reshape(txT, [1,-1])

        tyT = vectorize_matrix(ty)
        tyT = np.reshape(tyT, [1,-1])
       
        out = np.array([txT[0],tyT[0]])
        return out
    
    def visualize_kernels(self):
        colors = [
        np.array([1, 0, 0]),  # Red
        np.array([0, 1, 0]),  # Green
        np.array([0, 0, 1]),  # Blue
        np.array([1, 1, 0]),  # Yellow
        np.array([1, 0, 1]),  # Magenta
        np.array([0, 1, 1])   # Cyan
            ]
        # np.random.seed(42)  # Seed for reproducibility, you can remove this if you want truly random colors each time
        # colors = np.random.rand(len(self.Po_ls), 3)  # Generating a unique color (RGB) for each matrix

        # Normalize matrices to ensure values are within [0, 1] for valid color coding
        max_value = np.max([np.max(matrix) for matrix in self.Po_ls])

        # Initialize an empty matrix for the sum
        summed_image = np.zeros((*self.Po_ls[0].shape, 3))  # Assuming all matrices have the same shape

        # Loop through each matrix and its corresponding color
        for matrix in self.Po_ls:
            color_id = np.random.randint(0, len(colors))
            color = colors[color_id]
            # Normalize and color code the current matrix
            colored_matrix = (matrix / max_value)[:, :, None] * color[None, None, :]
            # Add the color-coded matrix to the sum
            summed_image += colored_matrix

        # Display the resulting image
        summed_image = np.clip(summed_image, 0, 1)
        plt.imshow(summed_image)
        plt.axis('off')  # Hide axis for better visualization
        plt.show()
        plt.savefig('kv'+str(len(self.Po_ls))+'.png')
        

    def load_kernels(self, directory):
        # Get the list of all files and directories
        self.kernel_ls = []
        all_files_and_dirs = os.listdir(directory)

        # Filter the list to get only files
        files = [f for f in all_files_and_dirs if os.path.isfile(os.path.join(directory, f))]
        files.sort()
        # print(files)
        for f in files:
            path = os.path.join(directory,f)
            data = scipy.io.loadmat(path)
            self.kernel_ls.append(data['output_resize'].flatten())

        self.kernel_ls = np.array(self.kernel_ls) 
        

      
    def get_K(self,cbf_lb= 0, clf_lb = 0): 
        print('wh', )
        wh = 155
        wv =1
        print('wh', wh)
        nx,ny = self.gs[0],self.gs[1]
        
        szp = int(nx*ny)
        
        szv = len(self.vrt)
       
        Mxv = self.v@self.A+self.cv*self.v
        Mxv = np.reshape(Mxv, [1,-1])
        
        # lim =3*10


        # kernel_ls = self.kernel_ls
        
        ncbf =  len(self.bH)
        m = gp.Model()
        # ###Defining the Optimization problem

        control_lim = 1
       
        


        nk = 100

        # kernel_ls =[self.U]
        K = m.addMVar((2,nk), ub = control_lim, lb = -control_lim, name = 'K')

        control_lim_b = 5
        Kb = m.addMVar((2,1), ub = control_lim_b, lb = -control_lim_b, name = 'Kb')
        infval = 10**6
       
        # Kp = m.addMVar((2,szp), ub = control_lim, lb = -control_lim, name = 'Kp')
        dv = m.addVar(ub = clf_lb, lb = -infval, name = 'dv')
        dh = m.addVar(ub = cbf_lb, lb = -infval, name = 'dh')
        
        # m.addConstr(dh<= dv)
   



        

        #### CLF related vars
       
        ls = m.addVar(name = 'lambda_s_v')
        lp = m.addMVar((1,len(self.bp)), ub = infval, lb = 0, name = 'lambda_p_v')

       
        lx = m.addMVar((1,len(self.bx)), lb= 0, ub = infval, name='lambda_x_v')
        
       

        # ###########

        
        
        ex = np.array([[1,0]])
        ey = np.array([[0,1]])
        tempv = (self.v@self.B)
        Mpv = 0
      
        Mpv = np.kron(tempv, self.kernel.T) @ K.reshape(-1)
        xj = np.reshape(self.cell.exit_vrt[0],(2,1))
        rv= -(self.cv*self.v@xj)+self.v@Kb
        
        ########CLF Constraints
        
       
        m.addConstr(-Mxv+lp@self.Ax2+lx@self.Ax== 0)
        m.addConstr((-lx@self.bx)[0,0] -(lp@self.bp)[0,0]+ls+rv[0][0]<= dv)
              
        for ix in range(szp):
            m.addConstr(0<= -Mpv[ix]+(lp@self.Ap)[0,ix]+ls)
           
        
         ###########CBF vars
      
        lsh = m.addMVar(ncbf, name = 'lambda_s_h')
        lph = m.addMVar((ncbf,len(self.bp)), ub = infval, lb = 0, name = 'lambda_p_h')

        
        lxh = m.addMVar((ncbf,len(self.bx)), lb= 0, ub = infval, name='lambda_x_h')
        ## rhoh = m.addMVar((ncbf,4,szp), lb= 0, ub = infval, name='rho_h')
      
        
        ## btxh = m.addMVar((ncbf,szp,len(self.bx)), lb= 0, ub = infval, name='beta_x_h')
        ## etah = m.addMVar((ncbf,szp,2,2), lb = 0, ub =infval, name ='eta_h')

        ########CBF constraints

        for ih in range(ncbf):
        # for ih in [2,1]:
        ## ih = 0
            rh = -self.ch*self.bH[ih] -self.AH[ih]@self.B@Kb
            
            Mxh = -self.AH[ih]@self.A-self.ch*self.AH[ih]
            temph = (-np.array([self.AH[ih]])@self.B)
            # Mph = 0
            
            Mph = np.kron(temph, self.kernel.T) @ K.reshape(-1) 

            
            m.addConstr(-Mxh+lph[ih]@self.Ax2+lxh[ih]@self.Ax == 0)
            
            
            ##m.addConstr(-Mxh+lph[ih]@self.Ax2+lxh[ih]@self.Ax+((rhoh[ih,0]-rhoh[ih,1])@np.ones(szp)*ex)[0]+((rhoh[ih,2]-rhoh[ih,3])@np.ones(szp)*ey)[0] == 0)
            ##m.addConstr(rhoh[ih,0]+rhoh[ih,1]==0)
            ##m.addConstr(rhoh[ih,2]+rhoh[ih,3]==0)
            

            m.addConstr(-(lxh[ih]@self.bx)[0]-lph[ih]@self.bp+lsh[ih]+rh<= dh)
            for ix in range(szp):
                m.addConstr(0<= -Mph[ix]+(lph[ih]@self.Ap)[ix]+lsh[ih])
                ##m.addConstr(btxh[ih][ix]@self.Ax+((etah[ih][ix][0,0]-etah[ih][ix][0,1])*ex)[0]+((etah[ih][ix][1,0]-etah[ih][ix][1,1])*ey)[0] ==0)
                ##m.addConstr(lzh[ih][0]-etah[ih][ix][0,0]-etah[ih][ix][0,1]==0)
                ##m.addConstr(lzh[ih][1]-etah[ih][ix][1,0]-etah[ih][ix][1,1]==0)
        
   
        m.update()
        m.setObjective(wv*dv+wh*dh,  GRB.MINIMIZE)
        m.update()
        # m.params.NonConvex = 2
        m.optimize()
        if m.Status == GRB.OPTIMAL:
            print('K=', K.X)
            self.K = K.X

            # self.Mp =kernel_sum.X
        
            # self.Mp =0
            # for i in range(len(kernel_ls)):
            #     self.Mp += K[i].X@kernel_ls[i]
            # # print('control gain', K.X)
            print('Kb', Kb.X)
            print('dv,dh =' ,dv.X,dh.X)
            self.Kb = Kb.X
            np.save(os.path.join(self.directory_save, 'K.npy'), self.K)
            np.save(os.path.join(self.directory_save, 'Kb.npy'), self.Kb)


    def get_K_old(self):
        ###calcualting gains with LP


        wh =1
        wv =1
        nx,ny = self.gs[0],self.gs[1]
        xj = self.l
        szp = int(nx*ny)

        szv = len(self.vrt)

        Mxv = self.v@self.A+self.cv*self.v
        Mxv = np.reshape(Mxv, [1,-1])

        # lim =3*10

        ncbf =  len(self.bH)
        m = gp.Model()
        # ###Defining the Optimization problem

        control_lim = 5
        
        nk = 100

        
        K = m.addMVar((2,nk), ub = control_lim, lb = -control_lim, name = 'K')
        # K2 = m.addMVar((2,2), ub = control_lim, lb = -control_lim, name = 'K2')
        # Kcos = m.addMVar((2,2), ub = control_lim, lb = -control_lim, name = 'Kcos')
        # Ksin = m.addMVar((2,2), ub = control_lim, lb = -control_lim, name = 'Ksin')
        infval = 10**6
        Kb = m.addMVar((2,1), ub = control_lim, lb = -control_lim, name = 'Kb')
        # Kp = m.addMVar((2,szp), ub = control_lim, lb = -control_lim, name = 'Kp')
        dv = m.addVar(ub = -0.01, lb = -infval, name = 'dv')
        dh = m.addVar(ub = -0.01, lb = -infval, name = 'dh')
        # m.addConstr(dh<= dv)
        # kernel_sum = K@self.U
        # kernel_sum = K@self.U+K2@self.U**2+Kcos@np.cos(self.U)
        
        

        lz = m.addMVar(2, ub = infval, lb =0)
        ls = m.addVar(name = 'lambda_s_v')
        lp = m.addMVar((1,len(self.bp)), ub = infval, lb = 0, name = 'lambda_p_v')


        lx = m.addMVar((1,len(self.bx)), lb= 0, ub = infval, name='lambda_x_v')
        rho = m.addMVar((4,szp), lb= 0, ub = infval, name='rho_v')
        # ry = m.addMVar((2,szp), lb= 0, ub = infval, name='rho_y')

        btx = m.addMVar((szp,len(self.bx)), lb= 0, ub = infval, name='beta_x_v')
        eta = m.addMVar((szp,2,2), lb = 0, ub =infval, name ='eta_v')





        # ###########



        ex = np.array([[1,0]])
        ey = np.array([[0,1]])
        tempv = (self.v@self.B)
        Mpv = np.kron(tempv, self.kernel.T) @ K.reshape(-1)
        # Mpv = tempv@(kernel_sum)
        
            
        xj = np.reshape(self.cell.exit_vrt[0],(2,1))
        rv= -(self.cv*self.v@xj)+self.v@Kb

        ########CLF Constraints

        m.addConstr(-Mxv+lp@self.Ax2+lx@self.Ax+(rho[0]-rho[1])@np.ones(szp)*ex+(rho[2]-rho[3])@np.ones(szp)*ey== 0)
        m.addConstr(rho[0]+rho[1]==0)
        m.addConstr(rho[2]+rho[3]==0)


        m.addConstr((-lx@self.bx)[0,0]+(rho[0]-rho[1])@(self.txT-self.l[0,0]*np.ones((1,szp)))[0]+(rho[2]-rho[3])@(self.tyT-self.l[1,0]*np.ones((1,szp)))[0] -(lp@self.bp)[0,0]+ls+self.sigma_max*lz[0]+self.sigma_max*lz[1]+rv[0][0]<= dv)


        for ix in range(szp):
            m.addConstr(-(btx[ix]@self.bx)[0]+(eta[ix,0,0]-eta[ix,0,1])*(-self.txT[0,ix]+self.l[0,0])+(eta[ix,1,0]-eta[ix,1,1])*(-self.tyT[0,ix]+self.l[1,0])<= -Mpv[ix]+(lp@self.Ap)[0,ix]+ls)
            m.addConstr(btx[ix]@self.Ax+((eta[ix][0,0]-eta[0,1])*ex)[0]+((eta[ix][1,0]-eta[ix][1,1])*ey)[0] ==0)
            m.addConstr(lz[0]-eta[ix][0,0]-eta[ix][0,1]==0)
            m.addConstr(lz[1]-eta[ix][1,0]-eta[ix][1,1]==0)

            ###########CBF vars
        lzh = m.addMVar((ncbf,2), ub = infval, lb =0)
        lsh = m.addMVar(ncbf, name = 'lambda_s_h')
        lph = m.addMVar((ncbf,len(self.bp)), ub = infval, lb = 0, name = 'lambda_p_h')


        lxh = m.addMVar((ncbf,len(self.bx)), lb= 0, ub = infval, name='lambda_x_h')
        rhoh = m.addMVar((ncbf,4,szp), lb= 0, ub = infval, name='rho_h')


        btxh = m.addMVar((ncbf,szp,len(self.bx)), lb= 0, ub = infval, name='beta_x_h')
        etah = m.addMVar((ncbf,szp,2,2), lb = 0, ub =infval, name ='eta_h')

        ########CBF constraints

        for ih in range(ncbf):
        # ih = 0
            rh = -self.ch*self.bH[ih] -self.AH[ih]@self.B@Kb
            
            Mxh = -self.AH[ih]@self.A-self.ch*self.AH[ih]
            temph = (-np.array([self.AH[ih]])@self.B)
            Mph = np.kron(temph, self.kernel.T) @ K.reshape(-1) 
            
            m.addConstr(-Mxh+lph[ih]@self.Ax2+lxh[ih]@self.Ax+((rhoh[ih,0]-rhoh[ih,1])@np.ones(szp)*ex)[0]+((rhoh[ih,2]-rhoh[ih,3])@np.ones(szp)*ey)[0] == 0)
            m.addConstr(rhoh[ih,0]+rhoh[ih,1]==0)
            m.addConstr(rhoh[ih,2]+rhoh[ih,3]==0)
            

            m.addConstr(-(lxh[ih]@self.bx)[0]+(rhoh[ih,0]-rhoh[ih,1])@(self.txT-self.l[0,0]*np.ones((1,szp)))[0]+(rhoh[ih,2]-rhoh[ih,3])@(self.tyT-self.l[1,0]*np.ones((1,szp)))[0]-lph[ih]@self.bp+lsh[ih]+self.sigma_max*lzh[ih][0]+self.sigma_max*lzh[ih][1]+rh<= dh)
            for ix in range(szp):
                m.addConstr(-btxh[ih][ix]@self.bx+(etah[ih][ix][0,0]-etah[ih][ix][0,1])*(-self.txT[0,ix]+self.l[0,0])+(etah[ih][ix][1,0]-etah[ih][ix][1,1])*(-self.tyT[0,ix]+self.l[1,0])<= -Mph[ix]+(lph[ih]@self.Ap)[ix]+lsh[ih])
                m.addConstr(btxh[ih][ix]@self.Ax+((etah[ih][ix][0,0]-etah[ih][ix][0,1])*ex)[0]+((etah[ih][ix][1,0]-etah[ih][ix][1,1])*ey)[0] ==0)
                m.addConstr(lzh[ih][0]-etah[ih][ix][0,0]-etah[ih][ix][0,1]==0)
                m.addConstr(lzh[ih][1]-etah[ih][ix][1,0]-etah[ih][ix][1,1]==0)


            m.update()
            m.setObjective(wv*dv+wh*dh,  GRB.MINIMIZE)
            m.update()
            # m.params.NonConvex = 2
            m.optimize()
        
        if m.Status == GRB.OPTIMAL:
            print('K=', K.X)
            self.K = K.X

            # self.Mp =kernel_sum.X
        
            # self.Mp =0
            # for i in range(len(kernel_ls)):
            #     self.Mp += K[i].X@kernel_ls[i]
            # # print('control gain', K.X)
            print('Kb', Kb.X)
            print('dv,dh =' ,dv.X,dh.X)
            self.Kb = Kb.X
            np.save(os.path.join(self.directory_save, 'K.npy'), self.K)
            np.save(os.path.join(self.directory_save, 'Kb.npy'), self.Kb)


        
class cell ():
    def __init__(self,Barrier, exit_Vertices,vrt ):
        self.bar = Barrier
        # self.wrd = world
        self.exit_vrt = exit_Vertices
        self.vrt = np.array(vrt)
        



    def check_in_polygon(self, p):
            """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """
            p = np.reshape(p,(1,2))
            from scipy.spatial import Delaunay
            if not isinstance(self.vrt,Delaunay):
                hull = Delaunay(self.vrt)

            return (hull.find_simplex(p)>=0)[0]
   


def gen_controller_all_orinetation(cell_i, directory_mat, directory_save, ch , cv, eps , sigma_max, dt  ):

    # dt = 0.001
    # # rate_maps = load_rate_map()
    # ch_ls = [100,1,1,10,1,1,100,10,1,100,1,100]
    # cv_ls = [0.1,1,1,1,1,1,0.1,0.1,1,0.9,1,1]

    # cell_ls = [c0]

    # sigma_max_ls = [0.01, 2, 0.1, 2, 2, 2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    # eps_ls = [0.01, 2, 1.5, 2.5, 2, 2, 1, 2, 2, 3, 2, 2]
    # # for i_cell in range(len(cell_ls)):

    # # i_cell = 0

    # # rate_maps_cell = rate_maps_select_triangles(i_cell,rate_maps)
    # # plt.imshow(np.sum(rate_maps_cell, axis= 0))
    # # plt.colorbar()
    # # plt.show()
    for deg in range(270, 360, 10):
            # A = np.zeros((2,2))
        A = np.zeros((2,2))
        # A = np.ones((2,2))*0.1
        B = np.eye((2))
        print('*************************deg=',deg,'*************************')
        s0=Control_cal(cell_i,A, B,dt,ch=ch,cv=cv ,sigma_max= sigma_max,eps=eps , grid_size_x=10,
                        grid_size_y=10, directory_mat =directory_mat+str(deg) , directory_save = directory_save+str(deg) )
        # print('***************************************cell=',i_cell )
        # print(cv_ls[i_cell], ch_ls[i_cell], sigma_max_ls[i_cell], eps_ls[i_cell])
        # s0.plot_cell()
        # plt.show()
        #

       
        s0.get_K()
        # s0.u_postion(0.0,0.0)
        

        # s0.check_Probability_constraints(0,0)
        s0.vector_F()
    # np.save('Mp'+str(i_cell), s0.Mp)
    # np.save('Kb'+str(i_cell), s0.Kb)



delta_x = 0.001


c0 = cell(
    Barrier=[
        [np.array([0.0, 1.2]), np.array([0.15, 1.2-delta_x])],
        [np.array([0.0, 1.2]), np.array([0.0+delta_x, 1.05])]
        ],
    exit_Vertices=[np.array([0, 1.05]), np.array([0.15, 1.05])],
    vrt=[np.array([0.0, 1.05]), np.array([0.15, 1.05]), np.array([0.15, 1.2]), np.array([0.0, 1.2])])

c1 = cell(
    Barrier=[
        [np.array([0.0, 1.05]), np.array([0.15, 1.05-delta_x])],
        [np.array([0.0, 1.05]), np.array([0.0, 0.9])],
        [np.array([0.0, 0.9]), np.array([0.15, 0.9+delta_x])]],
    exit_Vertices=[np.array([0.15, 1.05]), np.array([0.15, 0.90])],
    vrt=[np.array([0.0, 0.9]), np.array([0.15, 0.9]), np.array([0.15, 1.05]), np.array([0.0, 1.05])])

c2 = cell(
    Barrier=[
        [np.array([0.0, 0.9]), np.array([0.15, 0.9-delta_x])],
        [np.array([0.0, 0.9]), np.array([0.0, 0.75])],
        [np.array([0.0, 0.75]), np.array([0.15, 0.75+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.9]), np.array([0.15, 0.75])],
    vrt=[np.array([0.0, 0.75]), np.array([0.15, 0.75]), np.array([0.15, 0.9]), np.array([0.0, 0.9])])

c3 = cell(
    Barrier=[
        [np.array([0.0, 0.75]), np.array([0.15, 0.75-delta_x])],
        [np.array([0.0, 0.75]), np.array([0.0, 0.60])],
        [np.array([0.0, 0.60]), np.array([0.15, 0.60+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.75]), np.array([0.15, 0.60])],
    vrt=[np.array([0.0, 0.60]), np.array([0.15, 0.60]), np.array([0.15, 0.75]), np.array([0.0, 0.75])])

c4 = cell(
    Barrier=[
        [np.array([0.0, 0.60]), np.array([0.15, 0.60-delta_x])],
        [np.array([0.0, 0.60]), np.array([0.0, 0.45])],
        [np.array([0.0, 0.45]), np.array([0.15, 0.45+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.60]), np.array([0.15, 0.45])],
    vrt=[np.array([0.0, 0.45]), np.array([0.15, 0.45]), np.array([0.15, 0.60]), np.array([0.0, 0.60])])

c5 = cell(
    Barrier=[
        [np.array([0.0, 0.45]), np.array([0.15, 0.45-delta_x])],
        [np.array([0.0, 0.45]), np.array([0.0, 0.30])],
        [np.array([0.0, 0.30]), np.array([0.15, 0.30+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.45]), np.array([0.15, 0.30])],
    vrt=[np.array([0.0, 0.30]), np.array([0.15, 0.30]), np.array([0.15, 0.45]), np.array([0.0, 0.45])])

c6 = cell(
    Barrier=[
        [np.array([0.0, 0.30]), np.array([0.15, 0.30-delta_x])],
        [np.array([0.0, 0.30]), np.array([0.0, 0.15])],
        [np.array([0.0, 0.15]), np.array([0.15, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.30]), np.array([0.15, 0.15])],
    vrt=[np.array([0.0, 0.15]), np.array([0.15, 0.15]), np.array([0.15, 0.30]), np.array([0.0, 0.30])])

c7 = cell(
    Barrier=[
        [np.array([0.0, 0.15]), np.array([0.15, 0.15-delta_x])],
        [np.array([0.0, 0.15]), np.array([0.0, 0.0])],
        [np.array([0.0, 0.0]), np.array([0.15, 0.0+delta_x])]],
    exit_Vertices=[np.array([0.15, 0.15]), np.array([0.15, 0.0])],
    vrt=[np.array([0.0, 0.0]), np.array([0.15, 0.0]), np.array([0.15, 0.15]), np.array([0.0, 0.15])])


c8 = cell(
    Barrier=[
        [np.array([0.15, 1.2]), np.array([0.30, 1.2])],
        [np.array([0.15, 1.2]), np.array([0.15+delta_x, 1.05])],
        [np.array([0.30, 1.2]), np.array([0.30-delta_x, 1.05])]],
    exit_Vertices=[np.array([0.15, 1.05]), np.array([0.30, 1.05])],
    vrt=[np.array([0.15, 1.2]), np.array([0.15, 1.05]), np.array([0.30, 1.05]), np.array([0.3, 1.20])])

c9 = cell(
    Barrier=[
        [np.array([0.15, 1.05]), np.array([0.30, 1.05])],
        [np.array([0.15, 1.05]), np.array([0.15+delta_x, 0.90])],
        [np.array([0.30, 1.05]), np.array([0.30-delta_x, 0.90])]],
    exit_Vertices=[np.array([0.15, 0.90]), np.array([0.30, 0.90])],
    vrt=[np.array([0.15, 1.05]), np.array([0.15, 0.9]), np.array([0.30, 0.9]), np.array([0.3, 1.05])])


c10 = cell(
    Barrier=[
        [np.array([0.15, 0.90]), np.array([0.30, 0.90])],
        [np.array([0.15, 0.90]), np.array([0.15+delta_x, 0.75])],
        [np.array([0.30, 0.90]), np.array([0.30-delta_x, 0.75])]],
    exit_Vertices=[np.array([0.15, 0.75]), np.array([0.30, 0.75])],
    vrt=[np.array([0.15, 0.90]), np.array([0.15, 0.75]), np.array([0.30, 0.75]), np.array([0.3, 0.90])])


c11 = cell(
    Barrier=[
        [np.array([0.15, 0.75]), np.array([0.30, 0.75])],
        [np.array([0.15, 0.75]), np.array([0.15+delta_x, 0.60])],
        [np.array([0.30, 0.75]), np.array([0.30-delta_x, 0.60])]],
    exit_Vertices=[np.array([0.15, 0.60]), np.array([0.30, 0.60])],
    vrt=[np.array([0.15, 0.75]), np.array([0.15, 0.60]), np.array([0.30, 0.60]), np.array([0.3, 0.75])])


c12 = cell(
    Barrier=[
        [np.array([0.15, 0.60]), np.array([0.30, 0.60])],
        [np.array([0.15, 0.60]), np.array([0.15+delta_x, 0.45])],
        [np.array([0.30, 0.60]), np.array([0.30-delta_x, 0.45])]],
    exit_Vertices=[np.array([0.15, 0.45]), np.array([0.30, 0.45])],
    vrt=[np.array([0.15, 0.60]), np.array([0.15, 0.45]), np.array([0.30, 0.45]), np.array([0.3, 0.60])])


c13 = cell(
    Barrier=[
        [np.array([0.15, 0.45]), np.array([0.30, 0.45])],
        [np.array([0.15, 0.45]), np.array([0.15+delta_x, 0.30])],
        [np.array([0.30, 0.45]), np.array([0.30-delta_x, 0.30])]],
    exit_Vertices=[np.array([0.15, 0.30]), np.array([0.30, 0.30])],
    vrt=[np.array([0.15, 0.45]), np.array([0.15, 0.30]), np.array([0.30, 0.30]), np.array([0.3, 0.45])])


c14 = cell(
    Barrier=[
        [np.array([0.15, 0.30]), np.array([0.30, 0.30-delta_x])],
        [np.array([0.15, 0.30]), np.array([0.15, 0.15])],
        [np.array([0.15, 0.15]), np.array([0.30, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.30, 0.30]), np.array([0.30, 0.15])],
    vrt=[np.array([0.15, 0.15]), np.array([0.30, 0.15]), np.array([0.30, 0.30]), np.array([0.15, 0.30])])


# c14 = cell(
#     Barrier=[
#         [np.array([0.15, 0.30]), np.array([0.30, 0.30-delta_x])],
#         [np.array([0.15, 0.30]), np.array([0.15, 0.15])],
#         ],
#     exit_Vertices=[np.array([0.30, 0.30]), np.array([0.30, 0.15])],
#     vrt=[np.array([0.15, 0.15]), np.array([0.30, 0.15]), np.array([0.30, 0.30]), np.array([0.15, 0.30])])

c15 = cell(
    Barrier=[
        [np.array([0.15, 0.0]), np.array([0.15+delta_x, 0.15])],
        [np.array([0.15, 0.0]), np.array([0.30, 0.0])],
        [np.array([0.3, 0.0]), np.array([0.30-delta_x, 0.15])]],
    exit_Vertices=[np.array([0.15, 0.15]), np.array([0.30, 0.15])],
    vrt=[np.array([0.15, 0.0]), np.array([0.30, 0.0]), np.array([0.30, 0.15]), np.array([0.15, 0.15])])



c16 = cell(
    Barrier=[
        [np.array([0.30, 1.2]), np.array([0.45, 1.2])],
        [np.array([0.30, 1.2]), np.array([0.30+delta_x, 1.05])],
        [np.array([0.45, 1.2]), np.array([0.45-delta_x, 1.05])]],
    exit_Vertices=[np.array([0.30, 1.05]), np.array([0.45, 1.05])],
    vrt=[np.array([0.30, 1.2]), np.array([0.30, 1.05]), np.array([0.45, 1.05]), np.array([0.45, 1.20])])



c17 = cell(
    Barrier=[
        [np.array([0.30, 1.05-delta_x]), np.array([0.45, 1.05])],
        [np.array([0.45, 1.05]), np.array([0.45, 0.9])],
        [np.array([0.45, 0.90]), np.array([0.30, 0.90+delta_x])]],
    exit_Vertices=[np.array([0.30, 0.90]), np.array([0.30, 1.05])],
    vrt=[np.array([0.30, 1.05]), np.array([0.30, 0.90]), np.array([0.45, 0.90]), np.array([0.45, 1.05])])



c18 = cell(
    Barrier=[
        [np.array([0.30+delta_x, 0.90]), np.array([0.30, 0.75])],
        [np.array([0.30, 0.75]), np.array([0.45, 0.75])],
        [np.array([0.45, 0.75]), np.array([0.45-delta_x, 0.90])]],
    exit_Vertices=[np.array([0.30, 0.90]), np.array([0.45, 0.90])],
    vrt=[np.array([0.30, 0.90]), np.array([0.30, 0.75]), np.array([0.45, 0.75]), np.array([0.45, 0.90])])


c19 = cell(
    Barrier=[
        [np.array([0.30+delta_x, 0.75]), np.array([0.30, 0.60])],
        [np.array([0.30, 0.60]), np.array([0.45, 0.60])],
        [np.array([0.45, 0.60]), np.array([0.45-delta_x, 0.75])]],
    exit_Vertices=[np.array([0.30, 0.75]), np.array([0.45, 0.75])],
    vrt=[np.array([0.30, 0.75]), np.array([0.30, 0.60]), np.array([0.45, 0.60]), np.array([0.45, 0.75])])


c20 = cell(
    Barrier=[
        [np.array([0.30+delta_x, 0.60]), np.array([0.30, 0.45])],
        [np.array([0.30, 0.45]), np.array([0.45, 0.45])],
        [np.array([0.45, 0.45]), np.array([0.45-delta_x, 0.60])]],
    exit_Vertices=[np.array([0.30, 0.60]), np.array([0.45, 0.60])],
    vrt=[np.array([0.30, 0.60]), np.array([0.30, 0.45]), np.array([0.45, 0.45]), np.array([0.45, 0.60])])


c21 = cell(
    Barrier=[
        [np.array([0.30+delta_x, 0.45]), np.array([0.30, 0.30])],
        [np.array([0.30, 0.30]), np.array([0.45, 0.30])],
        [np.array([0.45, 0.30]), np.array([0.45-delta_x, 0.45])]],
    exit_Vertices=[np.array([0.30, 0.45]), np.array([0.45, 0.45])],
    vrt=[np.array([0.30, 0.45]), np.array([0.30, 0.30]), np.array([0.45, 0.30]), np.array([0.45, 0.45])])

c22 = cell(
    Barrier=[
        [np.array([0.30, 0.30]), np.array([0.45, 0.30-delta_x])],
        [np.array([0.30, 0.30]), np.array([0.30, 0.15])],
        [np.array([0.30, 0.15]), np.array([0.45, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.45, 0.30]), np.array([0.45, 0.15])],
    vrt=[np.array([0.30, 0.15]), np.array([0.45, 0.15]), np.array([0.45, 0.30]), np.array([0.30, 0.30])])



c23 = cell(
    Barrier=[
        [np.array([0.30, 0.0]), np.array([0.30+delta_x, 0.15])],
        [np.array([0.30, 0.0]), np.array([0.45, 0.0])],
        [np.array([0.45, 0.0]), np.array([0.45-delta_x, 0.15])]],
    exit_Vertices=[np.array([0.30, 0.15]), np.array([0.45, 0.15])],
    vrt=[np.array([0.30, 0.0]), np.array([0.45, 0.0]), np.array([0.45, 0.15]), np.array([0.30, 0.15])])


c24 = cell(
    Barrier=[
        [np.array([0.45, 1.2]), np.array([0.60, 1.2])],
        [np.array([0.45, 1.2]), np.array([0.45+delta_x, 1.05])],
        [np.array([0.60, 1.2]), np.array([0.60-delta_x, 1.05])]],
    exit_Vertices=[np.array([0.45, 1.05]), np.array([0.60, 1.05])],
    vrt=[np.array([0.45, 1.2]), np.array([0.45, 1.05]), np.array([0.60, 1.05]), np.array([0.60, 1.20])])



c25 = cell(
    Barrier=[
        [np.array([0.45, 1.05-delta_x]), np.array([0.60, 1.05])],
        [np.array([0.60, 1.05]), np.array([0.60, 0.9])],
        [np.array([0.60, 0.90]), np.array([0.45, 0.90+delta_x])]],
    exit_Vertices=[np.array([0.45, 0.90]), np.array([0.45, 1.05])],
    vrt=[np.array([0.45, 1.05]), np.array([0.45, 0.90]), np.array([0.60, 0.90]), np.array([0.60, 1.05])])

    
c26 = cell(
    Barrier=[
        [np.array([0.45, 0.90-delta_x]), np.array([0.60, 0.90])],
        [np.array([0.60, 0.90]), np.array([0.60, 0.75])],
        [np.array([0.60, 0.75]), np.array([0.45, 0.75+delta_x])]],
    exit_Vertices=[np.array([0.45, 0.75]), np.array([0.45, 0.90])],
    vrt=[np.array([0.45, 0.90]), np.array([0.45, 0.75]), np.array([0.60, 0.75]), np.array([0.60, 0.90])])


c27 = cell(
Barrier=[
    [np.array([0.45, 0.75-delta_x]), np.array([0.60, 0.75])],
    [np.array([0.60, 0.75]), np.array([0.60, 0.60])],
    [np.array([0.60, 0.60]), np.array([0.45, 0.60+delta_x])]],
exit_Vertices=[np.array([0.45, 0.60]), np.array([0.45, 0.75])],
vrt=[np.array([0.45, 0.75]), np.array([0.45, 0.60]), np.array([0.60, 0.60]), np.array([0.60, 0.75])])


c28 = cell(
Barrier=[
    [np.array([0.45, 0.60-delta_x]), np.array([0.60, 0.60])],
    [np.array([0.60, 0.60]), np.array([0.60, 0.45])],
    [np.array([0.60, 0.45]), np.array([0.45, 0.45+delta_x])]],
exit_Vertices=[np.array([0.45, 0.45]), np.array([0.45, 0.60])],
vrt=[np.array([0.45, 0.60]), np.array([0.45, 0.45]), np.array([0.60, 0.45]), np.array([0.60, 0.60])])



c29 = cell(
Barrier=[
    [np.array([0.45, 0.45-delta_x]), np.array([0.60, 0.45])],
    [np.array([0.60, 0.45]), np.array([0.60, 0.30])],
    [np.array([0.60, 0.30]), np.array([0.45, 0.30+delta_x])]],
exit_Vertices=[np.array([0.45, 0.30]), np.array([0.45, 0.45])],
vrt=[np.array([0.45, 0.45]), np.array([0.45, 0.30]), np.array([0.60, 0.30]), np.array([0.60, 0.45])])


c30 = cell(
    Barrier=[
        [np.array([0.45, 0.30]), np.array([0.60, 0.30-delta_x])],
        [np.array([0.45, 0.30]), np.array([0.45, 0.15])],
        [np.array([0.45, 0.15]), np.array([0.60, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.60, 0.30]), np.array([0.60, 0.15])],
    vrt=[np.array([0.45, 0.15]), np.array([0.60, 0.15]), np.array([0.60, 0.30]), np.array([0.45, 0.30])])


c31 = cell(
    Barrier=[
        [np.array([0.45, 0.0]), np.array([0.45+delta_x, 0.15])],
        [np.array([0.45, 0.0]), np.array([0.60, 0.0])],
        [np.array([0.60, 0.0]), np.array([0.60-delta_x, 0.15])]],
    exit_Vertices=[np.array([0.45, 0.15]), np.array([0.60, 0.15])],
    vrt=[np.array([0.45, 0.0]), np.array([0.60, 0.0]), np.array([0.60, 0.15]), np.array([0.45, 0.15])])


c32 = cell(
    Barrier=[
        [np.array([0.60+delta_x, 0.45]), np.array([0.60, 0.60])],
        [np.array([0.60, 0.60]), np.array([0.75, 0.60])],
        [np.array([0.75, 0.60]), np.array([0.75-delta_x, 0.45])]],
    exit_Vertices=[np.array([0.60, 0.45]), np.array([0.75, 0.45])],
    vrt=[np.array([0.60, 0.45]), np.array([0.60, 0.60]), np.array([0.75, 0.60]), np.array([0.75, 0.45])])


c33 = cell(
Barrier=[
    [np.array([0.60, 0.45-delta_x]), np.array([0.75, 0.45])],
    [np.array([0.75, 0.45]), np.array([0.75, 0.30])],
    [np.array([0.75, 0.30]), np.array([0.60, 0.30+delta_x])]],
exit_Vertices=[np.array([0.60, 0.30]), np.array([0.60, 0.45])],
vrt=[np.array([0.60, 0.45]), np.array([0.60, 0.30]), np.array([0.75, 0.30]), np.array([0.75, 0.45])])



c34 = cell(
    Barrier=[
        [np.array([0.60, 0.30]), np.array([0.75, 0.30-delta_x])],
        [np.array([0.60, 0.30]), np.array([0.60, 0.15])],
        [np.array([0.60, 0.15]), np.array([0.75, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.75, 0.30]), np.array([0.75, 0.15])],
    vrt=[np.array([0.60, 0.15]), np.array([0.75, 0.15]), np.array([0.75, 0.30]), np.array([0.60, 0.30])])


c35 = cell(
    Barrier=[
        [np.array([0.60, 0.0]), np.array([0.60+delta_x, 0.15])],
        [np.array([0.60, 0.0]), np.array([0.75, 0.0])],
        [np.array([0.75, 0.0]), np.array([0.75-delta_x, 0.15])]],
    exit_Vertices=[np.array([0.60, 0.15]), np.array([0.75, 0.15])],
    vrt=[np.array([0.60, 0.0]), np.array([0.75, 0.0]), np.array([0.75, 0.15]), np.array([0.60, 0.15])])


c36 = cell(
    Barrier=[
        [np.array([0.75+delta_x, 0.45]), np.array([0.75, 0.60])],
        [np.array([0.75, 0.60]), np.array([0.90, 0.60])],
        [np.array([0.90, 0.60]), np.array([0.90-delta_x, 0.45])]],
    exit_Vertices=[np.array([0.75, 0.45]), np.array([0.90, 0.45])],
    vrt=[np.array([0.75, 0.45]), np.array([0.75, 0.60]), np.array([0.90, 0.60]), np.array([0.90, 0.45])])


c37 = cell(
Barrier=[
    [np.array([0.75, 0.45-delta_x]), np.array([0.90, 0.45])],
    [np.array([0.90, 0.45]), np.array([0.90, 0.30])],
    [np.array([0.90, 0.30]), np.array([0.75, 0.30+delta_x])]],
exit_Vertices=[np.array([0.75, 0.30]), np.array([0.75, 0.45])],
vrt=[np.array([0.75, 0.45]), np.array([0.75, 0.30]), np.array([0.90, 0.30]), np.array([0.90, 0.45])])


c38 = cell(
    Barrier=[
        [np.array([0.75, 0.30]), np.array([0.90, 0.30-delta_x])],
        [np.array([0.75, 0.30]), np.array([0.75, 0.15])],
        [np.array([0.75, 0.15]), np.array([0.90, 0.15+delta_x])]],
    exit_Vertices=[np.array([0.90, 0.30]), np.array([0.90, 0.15])],
    vrt=[np.array([0.75, 0.15]), np.array([0.90, 0.15]), np.array([0.90, 0.30]), np.array([0.75, 0.30])])


c39 = cell(
    Barrier=[
        [np.array([0.75, 0.0]), np.array([0.75+delta_x, 0.15])],
        [np.array([0.75, 0.0]), np.array([0.90, 0.0])],
        [np.array([0.90, 0.0]), np.array([0.90-delta_x, 0.15])]],
    exit_Vertices=[np.array([0.75, 0.15]), np.array([0.90, 0.15])],
    vrt=[np.array([0.75, 0.0]), np.array([0.90, 0.0]), np.array([0.90, 0.15]), np.array([0.75, 0.15])])



c40 = cell(
    Barrier=[
        [np.array([0.90+delta_x, 0.45]), np.array([0.90, 0.60])],
        [np.array([0.90, 0.60]), np.array([1.05, 0.60])],
        [np.array([1.05, 0.60]), np.array([1.05-delta_x, 0.45])]],
    exit_Vertices=[np.array([0.90, 0.45]), np.array([1.05, 0.45])],
    vrt=[np.array([0.90, 0.45]), np.array([0.90, 0.60]), np.array([1.05, 0.60]), np.array([1.05, 0.45])])



c41 = cell(
Barrier=[
    [np.array([0.90, 0.45-delta_x]), np.array([1.05, 0.45])],
    [np.array([1.05, 0.45]), np.array([1.05, 0.30])],
    [np.array([1.05, 0.30]), np.array([0.90, 0.30+delta_x])]],
exit_Vertices=[np.array([0.90, 0.30]), np.array([0.90, 0.45])],
vrt=[np.array([0.90, 0.45]), np.array([0.90, 0.30]), np.array([1.05, 0.30]), np.array([1.05, 0.45])])



c42 = cell(
Barrier=[
    [np.array([0.90+delta_x, 0.30]), np.array([0.90, 0.15])],
    [np.array([0.90, 0.15]), np.array([1.05, 0.15])],
    [np.array([1.05, 0.15]), np.array([1.05-delta_x, 0.30])]],
exit_Vertices=[np.array([0.90, 0.30]), np.array([1.05, 0.30])],
vrt=[np.array([0.90, 0.30]), np.array([0.90, 0.15]), np.array([1.05, 0.15]), np.array([1.05, 0.30])])



c43 = cell(
Barrier=[
    [np.array([0.90+delta_x, 0.15]), np.array([0.90, 0.0])],
    [np.array([0.90, 0.0]), np.array([1.05, 0.0])],
    [np.array([1.05, 0.0]), np.array([1.05-delta_x, 0.15])]],
exit_Vertices=[np.array([0.90, 0.15]), np.array([1.05, 0.15])],
vrt=[np.array([0.90, 0.15]), np.array([0.90, 0.0]), np.array([1.05, 0.0]), np.array([1.05, 0.15])])


c44 = cell(
Barrier=[
    [np.array([1.05, 0.45+delta_x]), np.array([1.20, 0.45])],
    [np.array([1.20, 0.45]), np.array([1.20, 0.60])],
    [np.array([1.20, 0.60]), np.array([1.05, 0.60-delta_x])]],
exit_Vertices=[np.array([1.05, 0.60]), np.array([1.05, 0.45])],
vrt=[np.array([1.05, 0.45]), np.array([1.20, 0.45]), np.array([1.20, 0.60]), np.array([1.05, 0.60])])


c45 = cell(
Barrier=[
    [np.array([1.05, 0.30+delta_x]), np.array([1.20, 0.30])],
    [np.array([1.20, 0.30]), np.array([1.20, 0.45])],
    [np.array([1.20, 0.45]), np.array([1.05, 0.45-delta_x])]],
exit_Vertices=[np.array([1.05, 0.45]), np.array([1.05, 0.30])],
vrt=[np.array([1.05, 0.30]), np.array([1.20, 0.30]), np.array([1.20, 0.45]), np.array([1.05, 0.45])])

c46 = cell(
Barrier=[
    [np.array([1.05, 0.15+delta_x]), np.array([1.20, 0.15])],
    [np.array([1.20, 0.15]), np.array([1.20, 0.30])],
    [np.array([1.20, 0.30]), np.array([1.05, 0.30-delta_x])]],
exit_Vertices=[np.array([1.05, 0.30]), np.array([1.05, 0.15])],
vrt=[np.array([1.05, 0.15]), np.array([1.20, 0.15]), np.array([1.20, 0.30]), np.array([1.05, 0.30])])

c47 = cell(
Barrier=[
    [np.array([1.05, 0.0+delta_x]), np.array([1.20, 0.0])],
    [np.array([1.20, 0.0]), np.array([1.20, 0.0])],
    [np.array([1.20, 0.15]), np.array([1.05, 0.15-delta_x])]],
exit_Vertices=[np.array([1.05, 0.15]), np.array([1.05, 0.0])],
vrt=[np.array([1.05, 0.0]), np.array([1.20, 0.0]), np.array([1.20, 0.15]), np.array([1.05, 0.15])])



cell_ls = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31,
             c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47 ]



cell_ls_vis = cell_ls
if __name__ == '__main__':
    
    A = np.zeros((2,2))
    B = np.eye((2))

    i_cell = 14
    directory_mat = 'cells_kernels/c'+str(i_cell)+'/deg'
    directory_save =  'cells_controllers/c'+str(i_cell)+'/deg'
    print("###############################cell", str(i_cell))
    ###The most robust
    # gen_controller_all_orinetation(cell_ls[i_cell], directory_mat, directory_save, ch =0.65*10**-2, cv=10**-2, eps = 6*10**-2, sigma_max = 10**-6, dt = 0.001 )
    gen_controller_all_orinetation(cell_ls[i_cell], directory_mat, directory_save, ch =0.6*10**-2, cv=10**-2, eps = 6*10**-2, sigma_max = 10**-6, dt = 0.001 )
