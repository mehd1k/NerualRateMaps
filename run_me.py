import subprocess
import sys
import numpy as np

import matplotlib.pyplot as plt
import matlab.engine
from modified_script import generate_images
import atexit

# def call_image_generation_script(X, Y, HD, output):
#     script_path = 'modified_script.py'
#     subprocess.run([sys.executable, script_path, "--X", str(X), "--Y", str(Y), "--HD", str(HD), "--output", output])




# def call_matlab_function(mat_file_path, output_file_path):
   
#     # eng = matlab.engine.start_matlab()

#     # Call the MATLAB function and capture the output
#     output = eng.run(mat_file_path, output_file_path, nargout=1)
#     output = eng.run(mat_file_path, output_file_path, nargout=1)


    
#     # eng.quit()

#     return np.array(output)


   

def main():
    eng = matlab.engine.start_matlab()
    X_value, Y_value, HD_value = 0.0, 0.0,270

    generated_image_path = 'test'


    # Step 1: Generate the image (mat file)
    generate_images(X_value, Y_value, HD_value, generated_image_path)

    # Step 2: generated V1 with MATLAB
    
    
    output_file_path = 'exp1.png'
    mat_file_path = 'test.mat'
    # v1 = call_matlab_function(mat_file_path, output_file_path)
    v1 = eng.run(mat_file_path, output_file_path, nargout=1)

    # Display the processed image using matplotlib
    plt.imshow(v1, cmap='gray')
    plt.colorbar()
    plt.show()
    atexit.register(eng.quit)

if __name__ == "__main__":
    main()
    