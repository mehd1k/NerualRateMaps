function U = generate_V1_RSC_model_response3(image_data)
%
% This function takes the current image and model states (S: firing rates; 
%  U: membrane potentials) to generate the the current model states.
% The model first processes input image by LGN and V1 processing; and then
%  implements V1-RSC model (Lian et al. 2023, Journal of Neuroscience).
% S: 100 x 1 (model responses with the current image input)
% U: 100 x 1
S_past = zeros(100,1); % initiliaztion of the firing rates of 100 model cells
U_past = zeros(100,1); % initiliaztion of the membrane potentials of 100 model cells

plot_figure = 0;
%% Load trained model
load('model_trained_in_Lshape.mat', 'env', 'lgn', 'v1', 'v1_response_max', 'lca');
% Model loaded here has 100 model cells

%% LGN & V1 processing of the input image
img = reshape(image_data, env.fov_x, env.fov_y)';
lgn_response = lgn_processing(img, lgn);
[~, v1_response] = v1_processing(lgn_response, v1);
v1_response = v1_response / v1_response_max; % response scaled by a global factor

%% V1-RSC model response
[S, U, S_his, ~] = sparse_coding_by_LCA(...
    v1_response, lca.A, lca.lambda, lca.thresh_type, lca.U_eta, lca.s_max, lca.n_iter, lca.history_flag, S_past, U_past);

%% Plot figure if needed
if plot_figure == 1
    figure(32);
    subplot 231;
    imagesc(img, [0 1])
    colormap(gca, 'gray');
    title('image')
    axis image
    axis off
    
    subplot 232;
    imagesc(lgn_response)
    colormap(gca, 'gray');
    title('LGN processed image')
    axis image
    axis off
    colorbar
    
    subplot 233;
    imagesc(imresize(reshape(v1_response,v1.num_complex*v1.Nx,v1.Ny)',[v1.Ny v1.Nx]));
    colormap(gca, 'gray');
    title('V1 responses')
    axis image
    axis off
    colorbar
    
    subplot 234
    plot(S_his);
end