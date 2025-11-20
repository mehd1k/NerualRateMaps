function [simple_response, complex_response, v1] = v1_processing(input, v1)
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
% 'simple_response': a N * 1 vector that represents neural responses of N
%           simple cells over the whole visual field
% 'complex_response': a N/4 * 1 vector that represents neural responses of
%           N/4 complex cells over the whole visual field
% 
% Author: Yanbo Lian
% Date: 2024-02-05

[Ny, Nx] = size(input);
sz = v1.RF_size;
x0 = (sz+1)/2;
y0 = (sz+1)/2; % center of the Gabor filter
num_simple_cell = size(v1.Gr,2);

%% Step 1 - A: simple-complex connection; each complex cell has 4 simple cells with
% different spatial phases (0, 90, 180 and 270) but same orientation and
% spatial frequency
A = zeros(num_simple_cell, num_simple_cell/4);
for i_complex = 1 : num_simple_cell/4
    A(1+4*(i_complex-1):4*i_complex, i_complex) = 1;
end

%% Step 2 - processing in V1: simple cell responses
i = 1; % index of the location
Y = y0 : v1.stride(1): Ny-y0;
X = x0 : v1.stride(2) : Nx-x0+1;
for y = Y
    for x = X
        input_temp = reshape(input(y-y0+1:y+y0-1, x-x0+1:x+x0-1), sz*sz, 1);
        
        % simple_cell_responses: [num_simple_cell, num_position] matrix
        % that represent simple cell responses at location i
        simple_cell_responses(:, i) = max(input_temp' * v1.Gr, 0); % non-negative firing rates for simple cells
        i = i + 1;
    end
end

%% Step 3 - processing in V1: complex cell responses
% Each complex cell receives input from 4 simple cells at this location
% with the same orientation and spatial frequency but different spatial
% phases (0, 90, 180 and 270 degrees)
complex_cell_responses = A' * simple_cell_responses;

simple_response = reshape(simple_cell_responses, numel(simple_cell_responses), 1);
complex_response = reshape(complex_cell_responses, numel(complex_cell_responses), 1);

































