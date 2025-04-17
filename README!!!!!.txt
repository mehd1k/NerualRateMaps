"generate_V1_RSC_model_response.m" is a Matlab function that generates model response (a 100x1 vector in this case) in response to the current image.
"main.m" is an example of how to use it.

"generate_current_image_with_position.py" generates the image at a particular position (specified at lines 6 to 8). The image is saved as a 'current_image.mat' which is the input to the Matlab function "generate_V1_RSC_model_response.m".
Because I'm not that proficient in Python, after spending some time, I think it would be quicker for you to transform the script into a function of module that generates the current image with current postion as the input argument.

-------------
Yanbo Lian
20240607