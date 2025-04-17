%% Load image input and generate the corresponding model response
% load('current_image.mat', 'images_data');
load('test/test.mat', 'images_data');
% load('outputs2/mat/indx_84.png.mat', 'images_data');
img = images_data;
img = double(img) / 255; %scale it to 0-1

S = zeros(100, 1); % initiliaztion of the firing rates of 100 model cells
U = zeros(100, 1); % initiliaztion of the membrane potentials of 100 model cells

% current S, U should be the input arguments when computing the response at the next position
[S, U] = generate_V1_RSC_model_response(img, S, U);
S3 = generate_V1_RSC_model_response3(img)