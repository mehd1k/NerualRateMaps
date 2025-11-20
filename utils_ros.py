import numpy as np
from cell_configs import cell_ls, cell


def gen_grid_points(cell_id, num_points=10):
    """
    Generate grid points for a given cell.
    
    Args:
        cell_id: ID of the cell
        num_points: Number of points along each axis
        
    Returns:
        X, Y: Flattened arrays of grid point coordinates
    """
    cell = cell_ls[cell_id]
    margin = 0.01
    xmin = np.min(cell.vrt[:,0]) + margin
    xmax = np.max(cell.vrt[:,0]) - margin
    ymin = np.min(cell.vrt[:,1]) + margin
    ymax = np.max(cell.vrt[:,1]) - margin
    x_range = np.linspace(xmin, xmax, num_points)
    y_range = np.linspace(ymin, ymax, num_points)
    X, Y = np.meshgrid(x_range, y_range)
    X, Y = X.flatten(), Y.flatten()
    return X, Y
