mat_file_path = 'input.mat';
load(mat_file_path, 'images_data');
% function output = run(name_file, output_file_path)
    % name_file = 'test.mat';
    load(name_file, 'images_data');
    load('model_trained_in_Lshape.mat', 'env', 'lgn', 'v1', 'v1_response_max', 'lca');
    img = images_data;
    img = double(img) / 255;
    
    % Process the image
    img = reshape(img, env.fov_x, env.fov_y)';
    lgn_response = lgn_processing(img, lgn);
    [~, v1_response] = v1_processing(lgn_response, v1);
    v1_response = v1_response / v1_response_max;
    output = imresize(reshape(v1_response, v1.num_complex*v1.Nx, v1.Ny)', [v1.Ny v1.Nx]);
    figure();
    imagesc(output);
    colormap(gca, 'gray');
    
    % Save the plot
%     output_file_path = 'exp1.png';
    saveas(gcf, output_file_path);
% end