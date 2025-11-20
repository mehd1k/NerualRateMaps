function S = generate_V1_RSC_model_response2(images_data, lca)

%% Load trained model
% load()

%% Load image input
load('test.mat', 'images_data');
images_data = images_data(end,:);
images_data = double(images_data) / 255; %scale it to 0-1

% Parameters of the environment
env.x_size = 1.2; % unit: m
env.y_size = 1.2; % unit: m
env.fov_x = 150; % image size (horizontal view angle)
env.fov_y = 90; % image size (vertical view angle)

%% LGN processing model parameters
lgn.type = 'DDoG';
lgn.RF_size = 11;
lgn.sigma_c = 0.3;
lgn.sigma_s = 1.5 * lgn.sigma_c;
lgn.stride = 1;
lgn.Ic = exp(-R.^2/2/lgn.sigma_c^2);
lgn.Ic = normalize_matrix(reshape(lgn.Ic,lgn.RF_size*lgn.RF_size,1),'unit abs');
lgn.Ic = reshape(lgn.Ic, lgn.RF_size,lgn.RF_size);
lgn.Is = exp(-R.^2/2/lgn.sigma_s^2);
lgn.Is= normalize_matrix(reshape(lgn.Is,lgn.RF_size*lgn.RF_size,1),'unit abs');
lgn.Is = reshape(lgn.Is, lgn.RF_size,lgn.RF_size);
if isequal(lgn.type, 'DDoG')
    lgn.sigma_d = lgn.sigma_s;
    lgn.Id = exp(-R.^2/2/lgn.sigma_d^2);
    lgn.Id = normalize_matrix(reshape(lgn.Id,lgn.RF_size*lgn.RF_size,1),'unit abs');
    lgn.Id = reshape(lgn.Id, lgn.RF_size,lgn.RF_size);
    lgn.epsilon = 0e-10; % useful in some cases to avoid zero division
end

%% V1 processing model parameters
v1.RF_size = 17; % unit: degree; v1.RF_size+lgn.RF_size-1 is the real size of the RF;
v1.amplitude = 1;
v1.normalized = 1; % Normalize each Gabor fitler to have L2 norm equal to 1.
v1.orientations = 0:15:180-15; % unit: degree; 6 spatial orientations
v1.frequencies = 0.02:0.02:0.1; % unit: cycles per degree; 5 spatial frequencies;
v1.phases = 0:90:270; % unit: degree; 4 spatial phases
v1.stride = [7 7]; %the vertical and horizotal step size of convolution
v1.num_simple = length(v1.orientations)*length(v1.frequencies)*length(v1.phases);
v1.num_complex = v1.num_simple/4;
v1.Gr = zeros((v1.RF_size)^2, v1.num_simple);

i = 0; % index of Gabor filters at this location
x0 = (v1.RF_size+1)/2; y0 = (v1.RF_size+1)/2; % center of the Gabor filter
for theta = v1.orientations
    for frequency = v1.frequencies
        for phi = v1.phases
            i = i + 1;
            
            % Gr: [sz^2, num_simple_cell] matrix that represents all Gabor
            % filters at this location
            v1.Gr(:,i) = gabor_filter_pixel(v1.amplitude, x0, y0, frequency, theta, phi, v1.RF_size, v1.normalized);
        end
    end
end
v1.RF_size_real = v1.RF_size + lgn.RF_size - 1;

%% LGN & V1 processing of the input image
img = reshape(images_data, env.fov_x, env.fov_y)';
lgn_response = lgn_processing(img, lgn);
[~, v1_response] = v1_processing(lgn_response, v1);
v1_response = v1_response / images_V1_max; % response scaled by a global factor

[lgn.Ny, lgn.Nx] = size(lgn_response);
v1.Ny = length((v1.RF_size+1)/2 : v1.stride(1): lgn.Ny-(v1.RF_size+1)/2);
v1.Nx = length((v1.RF_size+1)/2 : v1.stride(2) : lgn.Nx-(v1.RF_size+1)/2+1);

max(v1_response(:))
max(lgn_response(:))
figure(32);
subplot 131;
imagesc(img, [0 1])
colormap(gca, 'gray');
title('image')
axis image
axis off

subplot 132;
imagesc(lgn_response)
colormap(gca, 'gray');
title('LGN processed image')
axis image
axis off
colorbar

subplot 133;
imagesc(imresize(reshape(v1_response,v1.num_complex*v1.Nx,v1.Ny)',[v1.Ny v1.Nx]));
colormap(gca, 'gray');
title('V1 responses')
axis image
axis off
colorbar
end

%% V1-RSC model
% [S_place, U_place, S_his, ~] = sparse_coding_by_LCA(...
%     v1_response, lca.A, lca.lambda, lca.thresh_type, lca.U_eta, lca.s_max, lca.n_iter, lca.history_flag, S_place, U_place);

