% generate_klt_images.m is the test file for the University of
% Pennsylvania's Coursera course Robotics:Perception, week 3 assignment, to
% track corners and project a virtual object into the scene
% Written for the University of Pennsylvania's Robotics:Perception course

% YOU SHOULDN'T NEED TO CHANGE THIS FOR THE ASSIGNMENT

% Initialize videoreader for video sequence (replace with your own if you
% want)
getFrame = @(i) imread(sprintf('data/apriltagims/image%03d.jpg',i));
num_frames = 166;
% Initialize the images
video_imgs = cell(num_frames, 1); 
% Process all the images
for i = 1:num_frames
    % Read the next video frame
    video_imgs{i} = getFrame(i);
end

%% Set of images to test on
test_images = 1:num_frames;
% To only test on images 1, 4 and 10, use the following line (you can edit
% it for your desired test images
% test_images = [1,4,10];
initial_points = 1.0e+02 * [
   1.981631469726562   3.165294189453125
   3.786268920898438   3.424402770996094
   4.036800842285157   1.686005859375000
   2.333528289794922   1.491907043457031 ];
corners = track_corners(video_imgs,initial_points);

%% Draw points in the image
generated_imgs = cell(num_frames, 1); 
for i = 1:num_frames
    draw_corners = {
      [corners(1,:,i) 5], ...
      [corners(2,:,i) 5], ...
      [corners(3,:,i) 5], ...
      [corners(4,:,i) 5], ...
    };
    generated_imgs{i} = insertShape(video_imgs{i}, ...
                        'FilledCircle',draw_corners{1},...
                        'Color','red');
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'FilledCircle',draw_corners{2},...
                        'Color','blue');
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'FilledCircle',draw_corners{3},...
                        'Color','green');
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'FilledCircle',draw_corners{4},...
                        'Color','black');
end


