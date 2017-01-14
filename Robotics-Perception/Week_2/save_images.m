function [] = save_images( images, im_name)
% save_images saves the set of images in images to png files with prefix
% im_name in folder warped_images
% Inputs:
%     images - Nx1 cell of images to be written to file
%     im_name - string for prefix of file name
% Written for the University of Pennsylvania's Robotics:Perception course

if nargin==1
    im_name = 'warped_img';
end

num_ima = length(images);

for i=1:num_ima
    imwrite(images{i}, ['warped_images/' im_name num2str(i) '.png']);
end

end

