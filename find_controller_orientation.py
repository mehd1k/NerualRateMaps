import os
import numpy as np
from gen_vec_field_from_perception import plot_vec
from gen_controller import cell, cell_ls
class control_gain_load():
    def __init__(self):
        self.dir_control_gains = 'cells_controllers'
       


    def load_matrix_from_folder(self,folder_path):
        """
        Load the matrix from the given folder.
        
        Args:
            folder_path (str): Path to the folder containing the matrix.
        
        Returns:
            np.ndarray: The matrix loaded from the folder.
        """
        Kb_path = os.path.join(folder_path, 'Kb.npy')  
        K_path = os.path.join(folder_path, 'K.npy')  
        if os.path.exists(Kb_path) and os.path.exists(K_path):
            return np.load(K_path) , np.load(Kb_path)  
        else:
            raise FileNotFoundError(f"No matrix file found in {folder_path}")

    def interpolate_matrices(self, matrix1, matrix2, degree1, degree2, target_degree):
        """
        Interpolate element-wise between two matrices based on the target orientation.
        
        Args:
            matrix1 (np.ndarray): The matrix corresponding to degree1.
            matrix2 (np.ndarray): The matrix corresponding to degree2.
            degree1 (float): The degree associated with matrix1.
            degree2 (float): The degree associated with matrix2.
            target_degree (float): The target orientation for interpolation.
        
        Returns:
            np.ndarray: The interpolated matrix.
        """
        # Compute the interpolation factor (linear interpolation formula)
        factor = (target_degree - degree1) / (degree2 - degree1)
        
        # Interpolate element-wise between the two matrices
        interpolated_matrix = (1 - factor) * matrix1 + factor * matrix2
        
        return interpolated_matrix

    def interpolate_contorlvec(self, cell_number, orientation):
        """
        Interpolate the matrix for a given orientation by loading only the two closest matrices.
        
        Args:
            base_folder (str): The path to the main folder containing 'deg' subfolders.
            orientation (float): The orientation angle for which the matrix is to be interpolated.
        
        Returns:
            np.ndarray: The interpolated matrix.
        """
        # Find the two closest degrees
        lower_degree = int(np.floor(orientation / 10) * 10)  # The degree lower or equal to the orientation
        upper_degree = int(np.ceil(orientation / 10) * 10)  # The degree higher or equal to the orientation
        
        if upper_degree > 350:
            upper_degree = 350  # Cap at 350
        
        if lower_degree < 0:
            lower_degree = 0  # Cap at 0
        
        # Load matrices from the two closest folders
        lower_folder = os.path.join(self.dir_control_gains,'c'+str(cell_number),f'deg{lower_degree}')
        upper_folder = os.path.join(self.dir_control_gains,'c'+str(cell_number), f'deg{upper_degree}')
        
        positions0, control_inputs_x0, control_inputs_y0= plot_vec(cell_number, cell_ls,lower_degree, plot=False)
        positions1, control_inputs_x1, control_inputs_y1 = plot_vec(cell_number, cell_ls,upper_degree, plot=False)
        
        K1,Kb1 = self.load_matrix_from_folder(lower_folder)
        K2,Kb2 = self.load_matrix_from_folder(upper_folder)
        
        # Perform interpolation
        if lower_degree == upper_degree:
            return positions0, control_inputs_x0, control_inputs_y0
        else:
            factor = (orientation - lower_degree) / (upper_degree - lower_degree)
            control_inputs_x_int = control_inputs_x0*(1-factor)+ control_inputs_x1*(factor)
            control_inputs_y_int = control_inputs_y0*(1-factor)+ control_inputs_y1*(factor)
            
            
            return positions0, control_inputs_x_int, control_inputs_y_int
        



    def interpolate_contorlgains(self, cell_number, orientation):
        """
        Interpolate the matrix for a given orientation by loading only the two closest matrices.
        
        Args:
            base_folder (str): The path to the main folder containing 'deg' subfolders.
            orientation (float): The orientation angle for which the matrix is to be interpolated.
        
        Returns:
            np.ndarray: The interpolated matrix.
        """
        # Find the two closest degrees
        
        lower_degree = int(np.floor(orientation / 10) * 10)  # The degree lower or equal to the orientation
        upper_degree = int(np.ceil(orientation / 10) * 10)  # The degree higher or equal to the orientation
        
        if upper_degree > 350:
            upper_degree = 350  # Cap at 350
        
        if lower_degree < 0:
            lower_degree = 0  # Cap at 0
        
        # Load matrices from the two closest folders
        lower_folder = os.path.join(self.dir_control_gains,'c'+str(cell_number),f'deg{lower_degree}')
        upper_folder = os.path.join(self.dir_control_gains,'c'+str(cell_number), f'deg{upper_degree}')
        
        K1,Kb1 = self.load_matrix_from_folder(lower_folder)
        K2,Kb2 = self.load_matrix_from_folder(upper_folder)
        
        # Perform interpolation
        if lower_degree == upper_degree:
            return K1, Kb1
        else:
            K_interpolated = self.interpolate_matrices(K1, K2, lower_degree, upper_degree, orientation)
            Kb_interpolated = self.interpolate_matrices(Kb1, Kb2, lower_degree, upper_degree, orientation)
            
            return K_interpolated, Kb_interpolated
        








if __name__ == '__main__':
    # Example usage:
    base_folder = '/path/to/robot/orientations'  # Replace with the actual path
    orientation = 45  # Example: interpolate matrix for 45 degrees

    cl = control_gain_load()
    folder_path = 'cells_controllers\c0\deg80'
    K, Kb = cl.load_matrix_from_folder(folder_path)
    print('K=', K)
    print('Kb=', Kb)
    # print(f"Interpolated matrix for orientation {orientation}°:\n", K,Kb)
