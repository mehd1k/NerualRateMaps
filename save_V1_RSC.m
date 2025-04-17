clear
clc
for i = 9:11
    % Define directories
    input_dir = ['cells_kernels/c' num2str(i) '/all/mat'];
    
    % output_dir_image = 'cells_kernels/c1/all/v1_response/images';
    output_dir_mat = ['cells_kernels/c' num2str(i) '/all/v1_response'];
    
    % Create the output directory if it does not exist
    % if ~exist(output_dir, 'dir')
    %     mkdir(output_dir);
    % end
    
    % Get a list of all .mat files in the input directory
    mat_files = dir(fullfile(input_dir, '*.mat'));
    
    % Load the trained model (assuming it's the same for all images)
    % load('model_trained_in_Lshape.mat', 'env', 'lgn', 'v1', 'v1_response_max', 'lca');
    S_old = zeros(100, 1); % initiliaztion of the firing rates of 100 model cells
    U_old = zeros(100, 1); % initiliaztion of the membrane potentials of 100 model cells
    % Loop through each .mat file
    for k = 1:length(mat_files)
        % Get the current file name and index
        [~, name, ~] = fileparts(mat_files(k).name);
        indx = name; % Assuming the name contains the index
    
        % Load the image data from the .mat file
        mat_file_path = fullfile(input_dir, mat_files(k).name);
        load(mat_file_path, 'images_data');
        img = images_data;
        img = double(img) / 255;
    
        % Process the image
        % img = reshape(img, env.fov_x,env.fov_x)';
            
      
        % current S, U should be the input arguments when computing the response at the next position
        [S_new, U_new] = generate_V1_RSC_model_response(img, S_old, U_old);
        S_new
        output_file_path_mat = fullfile(output_dir_mat, ['mat' indx '.mat']);
        save(output_file_path_mat, 'S_new');
        % S_old = S_new;
        % U_old = U_new;
    end
end
