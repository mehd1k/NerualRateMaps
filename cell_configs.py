import numpy as np
from scipy.spatial import Delaunay
        
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
            
            if not isinstance(self.vrt,Delaunay):
                hull = Delaunay(self.vrt)

            return (hull.find_simplex(p)>=0)[0]
        
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


c_test = cell(
    Barrier=[
        [np.array([0.15, 1.05]), np.array([0.45, 1.05])],
        [np.array([0.45, 1.05]), np.array([0.45-delta_x, 0.75])],
        [np.array([0.15, 1.05]), np.array([0.15+delta_x, 0.75])]
        ],
    exit_Vertices=[np.array([0.15, 0.75]), np.array([0.45, 0.75])],
    vrt=[np.array([0.15, 1.05]), np.array([0.45, 1.05]), np.array([0.45, 0.75]), np.array([0.15, 0.75])])


cell_ls = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31,
             c32, c33, c34, c35, c36, c37, c38, c39, c40, c41, c42, c43, c44, c45, c46, c47, c_test ]

