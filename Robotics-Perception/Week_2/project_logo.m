% project_logo.m is the test file for the University of
% Pennsylvania's Coursera course Robotics:Perception, week 2 assignment, to
% project a logo onto a given area in a target image
% Written for the University of Pennsylvania's Robotics:Perception course

% Note: You don't have to change this script for the assignment, but you
% can if you'd like to change the images or other parameters

% Load logo image. Replace with your image as desired.
logo_img = imread('images/logos/penn_engineering_logo.png');
% Generate logo points (they are just the outer corners of the image)
[logoy, logox, ~] = size(logo_img);
logo_pts = [0 0; logox 0; logox logoy; 0 logoy];

% Load the points that the logo corners will map onto in the main image
load data/BarcaReal_pts.mat
num_ima = size(video_pts, 3);

% Set of images to test on
test_images = 1:num_ima;
% To only test on images 1, 4 and 10, use the following line (you can edit
% it for your desired test images)
% test_images = [1,4,10];

num_test = length(test_images);

% Initialize the images
video_imgs = cell(num_test, 1);
projected_imgs = cell(num_test, 1);

% Process all the images
for i=1:num_test
    % Read the next video frame
    video_imgs{i} = imread(sprintf('images/barcaReal/BarcaReal%03d.jpg', i));
    
    % Find all points in the video frame inside the polygon defined by
    % video_pts
    [ interior_pts ] = calculate_interior_pts(size(video_imgs{i}),...
        video_pts(:,:,test_images(i)));
    
    % Warp the interior_pts to coordinates in the logo image
    warped_logo_pts = warp_pts(video_pts(:,:,test_images(i)),...
        logo_pts,...
        interior_pts);
    
    % Copy the RGB values from the logo_img to the video frame
    projected_imgs{i} = inverse_warping(video_imgs{i},...
        logo_img,...
        interior_pts,...
        warped_logo_pts); 
end