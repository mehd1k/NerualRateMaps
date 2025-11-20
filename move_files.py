import os
import shutil

# Source and destination directories
for i in range(8,12):

    source_dir = 'cells_kernels\\c'+str(i)+'\\all\\v1_response'
    destination_base_dir = 'cells_kernels\\c'+str(i)

    # Loop through each file in the source directory
    for filename in os.listdir(source_dir):
        if filename.startswith("matx") and "_HD" in filename:
            # Extract the HD value from the filename
            hd_value_str = filename.split("_HD")[1].split(".")[0]
            hd_value = float(hd_value_str)

            # Round HD value to the nearest 10
            rounded_hd = round(hd_value / 10) * 10

            # Create the corresponding folder name
            folder_name = f"deg{int(rounded_hd)}"
            destination_dir = os.path.join(destination_base_dir, folder_name)

            # Ensure the destination folder exists
            if not os.path.exists(destination_dir):
                os.makedirs(destination_dir)

            # Move the file to the corresponding folder
            source_file_path = os.path.join(source_dir, filename)
            destination_file_path = os.path.join(destination_dir, filename)

            shutil.move(source_file_path, destination_file_path)
            print(f"Moved {filename} to {destination_file_path}")
