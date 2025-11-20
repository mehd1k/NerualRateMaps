% mat_file_path = 'test/x0.01_y0.89_HD90.00';
% load(mat_file_path, 'images_data');
% img = images_data;
% img = double(img) / 255;

% Process the image
% img = reshape(img, env.fov_x,env.fov_x)';
img = zeros(90,150);
S_old = zeros(100, 1); % initiliaztion of the firing rates of 100 model cells
U_old = zeros(100, 1); % initiliaztion of the membrane potentials of 100 model cells
% current S, U should be the input arguments when computing the response at the next position
[S_new, U_new] = generate_V1_RSC_model_response(img, S_old, U_old);
