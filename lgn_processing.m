function lgn_response = lgn_processing(input, lgn)
% 
% This function generates responses of simple cells and complex cells in
% response to a static input image.
% 
% For the current parameters used in this script, a 120*160 image (19200 
% pixels) is represented by 79200 simple cells and 19800 complex cells.
% 
% Complex cell responses or simple cell responses(or both) can be used as
% the input to the hippocampal formation.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% 'input':  a Ny * Nx matrix that represents the raw input image Ny*Nx, 
%           where Nx and Ny are the size the image in x and y axis. For
%           Simon's visual image: Nx=160 and Ny = 120
% 
% Date: 2024-02-05

[Ny, Nx] = size(input);

if isequal(lgn.type, 'DDoG')
    input_lgn = imfilter(input,lgn.Ic-lgn.Is)./(imfilter(input,lgn.Id)+lgn.epsilon);
    input_lgn(isnan(input_lgn)) = 0;
else
    input_lgn = imfilter(input,lgn.Ic-lgn.Is);
end% DoG

%% Remove lgn cells around the boundaries & Downsampling
input_lgn = input_lgn((1+lgn.RF_size)/2:(Ny-(lgn.RF_size-1)/2),(1+lgn.RF_size)/2:(Nx-(lgn.RF_size-1)/2));
lgn_response = input_lgn(1:lgn.stride:end,1:lgn.stride:end);