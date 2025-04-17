import numpy as np
import matplotlib.pyplot as plt
import math
# from shapely.geometry import Point, Polygon
from scipy.spatial import Delaunay
import cvxpy as cp
import gurobipy as gp
from gurobipy import GRB
import ControllerCell as pt




def findcell(x, cell_ls):
    ls_flag=[]
    for i in range(len(cell_ls)):
        ls_flag.append(cell_ls[i].check_in_polygon(np.reshape(x,(1,2))))
    
    # print(ls_flag)
    return [i for i, x in enumerate(ls_flag) if x]



def traj_gen(mod,x_init,cell_ls,idi):
    traj =x_init
    

    x_old = x_init
    n_max_iter = 5000
    
    CD = pt.Discrertized_Linear_Controller( A, B, dt)
    A_dis,B_dis = CD()
   
    iter = 0
    while True:
    
        i_cell = findcell(x_old, cell_ls)[0]
        ci = cell_ls[i_cell]
        obs = pt.observation(ci.l, ci.eps/2, ci.sigma_max/2, ci.gs, ci.d)
        if mod ==0:
         Po = pt.vectorize_matrix(obs.obs(x_old))
        if mod ==1: 
            Po = pt.vectorize_matrix(obs.guass(x_old))

    
        u=ci.u(Po)
        # u = u/np.linalg.norm(u)*10
        speed = 30
        
       
        u = u/np.linalg.norm(u)*speed
            # ci.v@(x_old-np.reshape(ci.cell.exit_vrt[0],(2,1)))
            
        # elif ci.v@(x_old-ci.l)<0.1:
        #     i_cell = min(findcell(x_old, cell_ls)[0]+1,len(cell_ls)-1)
        #     ci = cell_ls[i_cell]
        #     obs = pt.observation(ci.l, ci.sigma_max/2, ci.eps/2, ci.gs, ci.d)
        #     Po = pt.vectorize_matrix(obs.obs(x_old))
        #     u=ci.u(Po)
        #     if np.linalg.norm(u) > maxsizeU:
        #         u = u/np.linalg.norm(u)*maxsizeU
    
        # print(np.linalg.norm(u))
        xk = A_dis@x_old+B_dis@u 
        # xk = x_old+(ci.A@x_old+ci.B@u)*ci.dt
        x_old = xk
   
        traj = np.hstack((traj, xk))
        if  iter > n_max_iter:
            print('max_iter')
            break

        
       
        iter += 1
    
    np.save('traj'+str(idi)+str(mod)+'.npy', traj)
    return traj




def visualization(mod,x_init,cell_ls, bars,id):
        traj = traj_gen(mod,x_init,cell_ls,id)



        X  = np.linspace(0,40,50)
        Y = np.linspace(0,40,50)
        ux_ls = []
        uy_ls = []
        # nx = self.gs[1]
        # ny = self.gs[0]
        xg =[]
        yg =[]
        cellX = []
        # self.cpox = np.zeros((len(Y),len(X)))
        # self.cpoy = np.zeros((len(Y),len(X)))
        for ix in range(len(X)):
            for iy in range(len(Y)):
                x =X[ix]
                y = Y[iy]
                x_pos = np.array([[x],[y]])
                icell = findcell(x_pos, cell_ls)
                if len(icell) > 0:
                    xg.append(x)
                    yg.append(y)
                    ci = cell_ls[icell[0]]
                    obs = pt.observation(ci.l, 2, 16, ci.gs, ci.d)
                    umax = 80
                    if mod == 0:
                        Po = pt.vectorize_matrix(obs.obs(x_pos))
                        sc = 0.5
                        
                        u =ci.u(Po)
                        
                        if np.linalg.norm(u)>umax:
                            u = u/np.linalg.norm(u)*umax
                        uc = np.copy(sc*u)
                        ux_ls.append(uc[0]) 
                        uy_ls.append(uc[1])
                    if mod == 1:
                        Po = pt.vectorize_matrix(obs.guass(x_pos))
                        u =ci.u(Po)
                        
                        # sc = 3
                        # if icell == 2 or 4:
                        #     u = sc*u
                        if np.linalg.norm(u)>umax:
                            u = u/np.linalg.norm(u)*umax
                        uc = np.copy(u)
                        ux_ls.append(uc[0]) 
                        uy_ls.append(uc[1])
        fig, ax = plt.subplots()    
        sc = 0.5
        ux_ls = sc* np.array(ux_ls)[:,0]
        uy_ls = sc* np.array(uy_ls)[:,0]
        xg = np.array(xg)
        yg = np.array(yg)
        for cell in cell_ls:
            for i in range(len(cell.vrt)-1):
                ax.plot([cell.vrt[i,0], cell.vrt[i+1,0]], [cell.vrt[i,1], cell.vrt[i+1,1]], color = 'gray')
            ax.plot([cell.vrt[0,0], cell.vrt[-1,0]], [cell.vrt[0,1], cell.vrt[-1,1]], color = 'gray')
            
        
        ax.quiver(xg,yg,ux_ls,uy_ls,angles='xy', scale_units='xy',color = 'gray', alpha = 0.4)
        ax.set_aspect('equal', adjustable='box')
        
        
       
        
       
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        ax.set_aspect('equal', adjustable='box')
        ax.plot(traj[0,:], traj[1,:] , color='green', linestyle='dashed', linewidth = 2)

        fig.savefig('traj_pt2.png', dpi= 600)

    
        return traj






def create_gif(mod,x_init,cell_ls, bars):
    traj = np.array(x_init)
    x_old = x_init
    n_max_iter = 250
    
    CD = pt.Discrertized_Linear_Controller( A, B, dt)
    A_dis,B_dis = CD()
   
    iter = 0
    while True:
    
        i_cell = findcell(x_old, cell_ls)[0]
        ci = cell_ls[i_cell]
        obs = pt.observation(ci.l, ci.eps/2, ci.sigma_max/2, ci.gs, ci.d)
        if mod ==0:
         Po = pt.vectorize_matrix(obs.obs(x_old))
        if mod ==1: 
            Po = pt.vectorize_matrix(obs.guass(x_old))

    
        u=ci.u(Po)
        # u = u/np.linalg.norm(u)*10
        speed = 450
        u = u/np.linalg.norm(u)*speed
        
        xk = A_dis@x_old+B_dis@u 
        x_old = xk
        
        
        traj = np.hstack((traj, xk))
        if  iter > n_max_iter:
            print('max_iter')
            break

        
       
       
        
        fig, ax = plt.subplots()
        # for cell in cell_ls:
        #                 for i in range(len(cell.vrt)-1):
        #                     ax.plot([cell.vrt[i,0], cell.vrt[i+1,0]], [cell.vrt[i,1], cell.vrt[i+1,1]], color = 'gray')
        #                 ax.plot([cell.vrt[0,0], cell.vrt[-1,0]], [cell.vrt[0,1], cell.vrt[-1,1]], color = 'gray')
                        
        ax.set_aspect('equal', adjustable='box')
        ax.plot(traj[0,:], traj[1,:] , color='green', linestyle='dashed', linewidth = 2)
        ax.scatter(xk[0], xk[1] ,color = 'blue')
        print('iter == ', iter)
    
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        

        fig.savefig('figures/traj/traj'+str(iter)+'.png', dpi= 200)
        plt.close('all')




        iter += 1
    
    np.save('figures/traj/traj_ls.npy', traj)
      
        
                     
                    
    



def grid_cell(cell_ls,bars):
        fig, ax = plt.subplots() 
        for cell in cell_ls:
            for i in range(len(cell.vrt)-1):
                ax.plot([cell.vrt[i,0], cell.vrt[i+1,0]], [cell.vrt[i,1], cell.vrt[i+1,1]], color = 'gray')
            ax.plot([cell.vrt[0,0], cell.vrt[-1,0]], [cell.vrt[0,1], cell.vrt[-1,1]], color = 'gray')
            
        
 
        
        
       
        
       
        for env_vrt in bars:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
            # fig.show()
        ax.set_aspect('equal', adjustable='box')
   

        fig.savefig('cells.png', dpi= 600)

    
    

    
def vecfield(mod,cell_ls,xmin,xmax,ymin,ymax,Barriers):
     
        CD = pt.Discrertized_Linear_Controller(cell_ls[0].A, cell_ls[0].B, cell_ls[0].dt)
        A_dis, B_dis = CD()
        X  = np.linspace(xmin,xmax,50)
        Y = np.linspace(ymin,ymax,50)
        ux_ls = []
        uy_ls = []
        # nx = self.gs[1]
        # ny = self.gs[0]
        xg =[]
        yg =[]
        cellX = []
        # self.cpox = np.zeros((len(Y),len(X)))
        # self.cpoy = np.zeros((len(Y),len(X)))
        for ix in range(len(X)):
            for iy in range(len(Y)):
                x =X[ix]
                y = Y[iy]
                x_pos = np.array([[x],[y]])
                icell = findcell(x_pos, cell_ls)
                if len(icell) > 0:
                    xg.append(x)
                    yg.append(y)
                    ci = cell_ls[icell[0]]
                    obs = pt.observation(ci.l, 2, 16, ci.gs, ci.d)
                    umax = 80
                    if mod == 0:
                        Po = pt.vectorize_matrix(obs.obs(x_pos))
                        sc = 0.5
                        
                        u =ci.u(Po)
                        
                        if np.linalg.norm(u)>umax:
                            u = u/np.linalg.norm(u)*umax
                        uc = np.copy(sc*u)
                        ux_ls.append(uc[0]) 
                        uy_ls.append(uc[1])
                    if mod == 1:
                        Po = pt.vectorize_matrix(obs.guass(x_pos))
                        u =ci.u(Po)
                        
                        # sc = 3
                        # if icell == 2 or 4:
                        #     u = sc*u
                        if np.linalg.norm(u)>umax:
                            u = u/np.linalg.norm(u)*umax
                        uc = np.copy(u)
                        ux_ls.append(uc[0]) 
                        uy_ls.append(uc[1])
        fig, ax = plt.subplots()    
        sc = 0.5
        ux_ls = sc* np.array(ux_ls)[:,0]
        uy_ls = sc* np.array(uy_ls)[:,0]
        xg = np.array(xg)
        yg = np.array(yg)
        for cell in cell_ls:
            for i in range(len(cell.vrt)-1):
                ax.plot([cell.vrt[i,0], cell.vrt[i+1,0]], [cell.vrt[i,1], cell.vrt[i+1,1]], color = 'gray')
            ax.plot([cell.vrt[0,0], cell.vrt[-1,0]], [cell.vrt[0,1], cell.vrt[-1,1]], color = 'gray')
            
        for env_vrt in Barriers:
            for i in range(len(env_vrt)-1):
                ax.plot([env_vrt[i][0], env_vrt[i+1][0]], [env_vrt[i][1], env_vrt[i+1][1]], color = 'red')

            ax.plot([env_vrt[0][0], env_vrt[-1][0]], [env_vrt[0][1], env_vrt[-1][1]], color = 'red')
          
        ax.quiver(xg,yg,ux_ls,uy_ls,angles='xy', scale_units='xy',color = 'gray', alpha = 0.4)
        ax.set_aspect('equal', adjustable='box')
        fig.show()
        fname = 'vec_field_mod'+str(mod)+'.png'
        plt.show()
        fig.savefig(fname, dpi = 600)
        # np.save('vecF.npy', [xg,yg,ux_ls,uy_ls])







# A = np.zeros((2,2))
A = np.zeros((2,2))
# A = np.ones((2,2))*0.1
B = np.eye((2))






########square cells

c0 = pt.cell(Barrier=[[np.array([1,30]), np.array([1,40])], [np.array([1,40]), np.array([10, 40])] , [np.array([10,40]), np.array([10, 30])]], exit_Vertices=[ np.array([10, 30]), np.array([1,30])], vrt= [ np.array([1, 30]), np.array([1,40]), np.array([10, 40]), np.array([10, 30])  ],landmark_ls=np.array([10, 30]))
c1 = pt.cell(Barrier=[[np.array([1,40]), np.array([1,20])], [np.array([10,40]), np.array([10, 20])] , [np.array([1,40]), np.array([10, 40])]], exit_Vertices=[ np.array([1, 20]), np.array([10,20])], vrt= [ np.array([1, 20]), np.array([10,20]), np.array([10, 40]), np.array([1, 40])  ],landmark_ls=np.array([10, 20]))
c2 = pt.cell(Barrier=[[np.array([10, 10]), np.array([10,20])], [np.array([10,20]), np.array([1,20])], [ np.array([1,20]), np.array([1,10])] ], exit_Vertices= [np.array([1,10]), np.array([10,10])], vrt= [np.array([1,10]), np.array([10,10]), np.array([10,20]), np.array([1,20]) ], landmark_ls= [ np.array([10,20])] )
c3 = pt.cell(Barrier=[[np.array([10, 10]), np.array([1,10])], [ np.array([1,10]), np.array([1,1])], [ np.array([1,1]), np.array([10,1])] ], exit_Vertices= [np.array([10,1]), np.array([10,10])], vrt= [np.array([10,1]), np.array([10,10]), np.array([1,10]), np.array([1,1]) ], landmark_ls= [ np.array([10,10])] )
c4 = pt.cell(Barrier=[[np.array([10, 1]), np.array([10,10])], [ np.array([10,10]), np.array([20,10])], [ np.array([10,1]), np.array([20,1])] ], exit_Vertices= [np.array([20,1]), np.array([20,10])], vrt= [np.array([10,1]), np.array([10,10]), np.array([20,10]), np.array([20,1]) ], landmark_ls= [ np.array([20,10])] )
c5 = pt.cell(Barrier=[[np.array([30, 1]), np.array([20,1])], [ np.array([20,1]), np.array([20,10])], [ np.array([20,10]), np.array([30,10])] ], exit_Vertices= [np.array([30,10]), np.array([30,1])], vrt= [np.array([30,1]), np.array([20,1]), np.array([20,10]), np.array([30,10]) ], landmark_ls= [ np.array([20,10])] )
c6 = pt.cell(Barrier=[[np.array([40, 10]), np.array([40,1])], [ np.array([40,1]), np.array([30,1])], [ np.array([30,1]), np.array([30, 10])] ], exit_Vertices= [np.array([30,10]), np.array([40,10])], vrt= [np.array([30,10]), np.array([30,1]), np.array([40,1]), np.array([40,10]) ], landmark_ls= [ np.array([30,10])] )
c7 = pt.cell(Barrier=[[np.array([30, 10]), np.array([40,10])], [ np.array([40,10]), np.array([40,20])], [ np.array([40,20]), np.array([30, 20])] ], exit_Vertices= [np.array([30,20]), np.array([30,10])], vrt= [np.array([30, 10]), np.array([40,10]), np.array([40,20]), np.array([30,20]) ], landmark_ls= [ np.array([30,10])] )
c8 = pt.cell(Barrier=[[np.array([20, 10]), np.array([30,10])], [ np.array([30,10]), np.array([30,20])], [ np.array([30,20]), np.array([20, 20])] ], exit_Vertices= [np.array([20,20]), np.array([20,10])], vrt= [np.array([20, 10]), np.array([30,10]), np.array([30,20]), np.array([20,20]) ], landmark_ls= [ np.array([20,10])] )
c9 = pt.cell(Barrier=[[np.array([20, 20]), np.array([20,10])], [ np.array([20,10]), np.array([10,10])], [ np.array([10,10]), np.array([10,20])] ], exit_Vertices= [np.array([10,20]), np.array([20,20])], vrt= [np.array([20,20]), np.array([20,10]), np.array([10,10]), np.array([10,20]) ], landmark_ls= [ np.array([10,20])] )
c10 = pt.cell(Barrier=[[np.array([10, 30]), np.array([10,20])], [ np.array([10,20]), np.array([20,20])], [ np.array([20,20]), np.array([20,30])] ], exit_Vertices= [np.array([10,30]), np.array([20,30])], vrt= [np.array([10,30]), np.array([10,20]), np.array([20,20]), np.array([20,30]) ], landmark_ls= [ np.array([10,20])] )
c11 = pt.cell(Barrier=[[np.array([10, 30]), np.array([20,30])], [ np.array([20,30]), np.array([20,40])], [ np.array([20,40]), np.array([10,40])] ], exit_Vertices= [np.array([10,40]), np.array([10,30])], vrt= [np.array([10,30]), np.array([20,30]), np.array([20,40]), np.array([10,40]) ], landmark_ls= [ np.array([10,30])] )







dt = 0.001
rate_maps = pt.load_rate_map()
ch_ls = [1,1,1,1,1,1,1,1,1,1,1,1]
cv_ls = [1,1,1,1,1,1,1,1,1,1,1,1]

cell_ls = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]





############Triangles cell

# c0 = pt.cell(Barrier=[[np.array([10,30]), np.array([20,40])], [np.array([20,40]), np.array([0, 40])] ], exit_Vertices=[ np.array([10, 30]), np.array([0,40])], vrt= [ np.array([10, 30]), np.array([20,40]), np.array([0, 40]) ],landmark_ls=np.array([10, 30]))
# c1 = pt.cell(Barrier=[[np.array([0,40]), np.array([10,30])], [np.array([0,40]), np.array([0, 20])] ], exit_Vertices=[ np.array([10, 30]), np.array([0,20])], vrt= [ np.array([0, 20]), np.array([10,30]), np.array([0, 40]) ],landmark_ls=np.array([10, 30]))
# c2 = pt.cell(Barrier=[[np.array([0,20]), np.array([10,30])], [np.array([10,30]), np.array([10, 10])], [np.array([0,30]), np.array([0, 10])] ], exit_Vertices=[ np.array([0, 20]), np.array([10,10])], vrt= [ np.array([0, 20]), np.array([10,30]), np.array([10, 10]) ],landmark_ls=np.array([10, 30]))
# c3 = pt.cell(Barrier=[[np.array([0, 0]), np.array([0, 20])], [np.array([0, 20]), np.array([10, 10])], [np.array([0,0]), np.array([20,0])] ], exit_Vertices=[ np.array([0, 0]), np.array([10,10])], vrt= [ np.array([0, 0]), np.array([10,10]), np.array([0, 20]) ],landmark_ls=np.array([10, 10]))
# c4 = pt.cell(Barrier=[[np.array([0, 0]), np.array([10, 10])], [np.array([0, 0]), np.array([20, 0])], [np.array([0,0]), np.array([0,20])] ], exit_Vertices=[ np.array([10, 10]), np.array([20,0])], vrt= [ np.array([0, 0]), np.array([20, 0]), np.array([10, 10]) ],landmark_ls=np.array([10, 10]))
# c5 = pt.cell(Barrier=[[np.array([30, 10]), np.array([10, 10])], [np.array([10, 10]), np.array([20, 0])], [np.array([0,0]), np.array([30,0])] ], exit_Vertices=[ np.array([20, 0]), np.array([30, 10])], vrt= [ np.array([10, 10]), np.array([20, 0]), np.array([30, 10]) ],landmark_ls=np.array([30, 10]))
# c6 = pt.cell(Barrier=[[np.array([20, 0]), np.array([40, 0])], [np.array([20, 0]), np.array([30, 10])], [np.array([40 , 0]), np.array([40,20])] ], exit_Vertices=[ np.array([40, 0]), np.array([30, 10])], vrt= [ np.array([30, 10]), np.array([20, 0]), np.array([40, 0]) ],landmark_ls=np.array([30, 10]))
# c7 = pt.cell(Barrier=[[np.array([30, 10]), np.array([40, 0])], [np.array([40, 0]), np.array([40, 20])], [np.array([40, 20]), np.array([20, 20])] ], exit_Vertices=[ np.array([30, 10]), np.array([40, 20])], vrt= [ np.array([30, 10]), np.array([40, 0]), np.array([40, 20]) ],landmark_ls=np.array([30, 10]))

# c8 = pt.cell(Barrier=[[np.array([30, 10]), np.array([40, 20])], [np.array([40, 20]), np.array([20, 20])]], exit_Vertices=[ np.array([30, 10]), np.array([20, 20])], vrt= [ np.array([30, 10]), np.array([40, 20]), np.array([20, 20]) ],landmark_ls=np.array([30, 10]))
# c9 = pt.cell(Barrier=[[np.array([20, 20]), np.array([30, 10])], [np.array([30, 10]), np.array([10, 10])] , [np.array([20, 20]), np.array([40, 20])]], exit_Vertices=[ np.array([10, 10]), np.array([20, 20])], vrt= [ np.array([30, 10]), np.array([20, 20]), np.array([10, 10]) ],landmark_ls=np.array([30, 10]))

# c10 = pt.cell(Barrier=[[np.array([10, 10]), np.array([20, 20])], [np.array([10, 10]), np.array([10, 20])], [np.array([20, 20]), np.array([20, 10])] ], exit_Vertices=[ np.array([20, 20]), np.array([10, 30])], vrt= [ np.array([20, 20]), np.array([10, 10]), np.array([10, 30]) ],landmark_ls=np.array([10, 10]))
# c11 = pt.cell(Barrier=[[np.array([10, 30]), np.array([20, 20])], [np.array([20, 20]), np.array([20, 40])], [np.array([20, 40]), np.array([30, 40])] ], exit_Vertices=[ np.array([10, 30]), np.array([20, 40])], vrt= [ np.array([10, 30]), np.array([20, 20]), np.array([20, 40]) ],landmark_ls=np.array([10, 30]))









# dt = 0.001
# rate_maps = pt.load_rate_map()
# ch_ls = [1,1,1,10,1,1,100,10,1,100,1,100]
# cv_ls = [1,1,1,1,1,1,0.1,0.1,1,0.9,1,1]

# cell_ls = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]

sigma_max_ls = [2, 2, 0.1, 2, 2, 2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

eps_ls = [2, 2, 1.5, 2.5, 2, 2, 1, 2, 2, 3, 2, 2]

dt = 0.001






# A = np.zeros((2,2))
A = np.zeros((2,2))
# A = np.ones((2,2))*0.1
B = np.eye((2))

CD = pt.Discrertized_Linear_Controller(A, B, dt)
A_dis, B_dis = CD()


cntcal=[]
w0 = np.array([0,0])
w1 = np.array([40,0])
w2 = np.array([40,20])
w3 = np.array([20,20])
w4 = np.array([20, 40])
w5 = np.array([0,40])
world = [w0, w1, w2, w3, w4, w5]
#######Triangles
# for i_cell in range(len(cell_ls)):
    
#     rate_maps_cell = pt.rate_maps_select_triangles(i_cell,rate_maps)
#     s=pt.Control_cal(cell_ls[i_cell],A, B,dt,ch=ch_ls[i_cell],cv=cv_ls[i_cell] ,sigma_max= sigma_max_ls[i_cell],eps=eps_ls[i_cell] , rate_maps_cell= rate_maps_cell, grid_size_x=rate_maps_cell.shape[1], grid_size_y=rate_maps_cell.shape[2] )



#     s.Mp = np.load('Mp'+str(i_cell)+'.npy')
#     s.Kb = np.load('Kb'+str(i_cell)+'.npy')
#     cntcal.append(s)
###squares
for i_cell in range(len(cell_ls)):
    
    rate_maps_cell = pt.rate_maps_select(i_cell,rate_maps)
    s=pt.Control_cal(cell_ls[i_cell],A, B,dt,ch=ch_ls[i_cell],cv=cv_ls[i_cell] ,sigma_max= sigma_max_ls[i_cell],eps=eps_ls[i_cell] , rate_maps_cell= rate_maps_cell, grid_size_x=rate_maps_cell.shape[1], grid_size_y=rate_maps_cell.shape[2] )



    s.Mp = np.load('square gains/Mp'+str(i_cell)+'.npy')
    s.Kb = np.load('square gains/Kb'+str(i_cell)+'.npy')
    cntcal.append(s)

x_init_ls = [np.array([[5],[37]])]


x_init_ls = [np.array([[5],[37]])]


# for idi in range(len(x_init_ls)):
#     visualization(0,x_init_ls[idi],cntcal, [world],idi)


# # grid_cell(cntcal, [world])
# plt.show()
# vecfield(0,cntcal,0 ,40, 0, 40,[world] )

create_gif(0,x_init_ls[0],cntcal, [world])

# ci = cntcal[1]
# x = np.reshape(ci.center, (2,1))
# x = np.array([[60],[75]])
# obs = pt.observation(np.reshape(world[1],(2,1)), 3, 16, ci.gs, ci.d)
# P_guass = obs.guass(x)
# P_delta = obs.obs(x)
# fig,ax = plt.subplots(1,2)
# im0 = ax[0].imshow(P_delta, cmap='bone')
# im1 = ax[1].imshow(P_guass, cmap='bone')
# # fig.colorbar(im0, orientation='vertical')
# # fig.colorbar(im1, orientation='vertical')
# fig.savefig('p'+str(x[0][0])+str(x[1][0])+'.png', dpi = 600)