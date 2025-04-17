import os


base_directory = 'cells_kernels'
# for i in range(12, 13):
#     folder_name = f"c{i}"
#     folder_path = os.path.join(base_directory, folder_name)
    
#     ####Create the folder if it doesn't exist
#     if not os.path.exists(folder_path):

#         os.makedirs(folder_path)
        
####Define the base directory where you want to create the folders
base_directory_ls = ['cells_controllers', 'cells_kernels_images', 'cells_kernels', 'lipt_test_line' ]

for j in range(47,48):
    for bd in base_directory_ls:


        base_directory = bd+"//c"+str(j)  # Replace with your desired path

        ###Loop to create folders from deg0 to deg360
        for i in range(0, 360, 10):
            folder_name = f"deg{i}"
            folder_path = os.path.join(base_directory, folder_name)
            
            # Create the folder if it doesn't exist
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
                print(f"Created folder: {folder_path}")
            else:
                print(f"Folder already exists: {folder_path}")





