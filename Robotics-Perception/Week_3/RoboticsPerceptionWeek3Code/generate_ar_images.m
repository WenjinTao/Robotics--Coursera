% generate_ar_images.m is the test file for the University of
% Pennsylvania's Coursera course Robotics:Perception, week 3 assignment, to
% project a virtual object into the scene
% Written for the University of Pennsylvania's Robotics:Perception course

% YOU SHOULDN'T NEED TO CHANGE THIS FOR THE ASSIGNMENT

% Initialize videoreader for video sequence (replace with your own if you
% want)
getFrame = @(i) imread(sprintf('data/apriltagims/image%03d.jpg',i));
num_frames = 166;
% Initialize the images
video_imgs = cell(num_frames, 1); 
% Process all the images
i = 1;
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
num_frames = 166;
generated_imgs = cell(num_frames, 1); 
% for i = 1:num_frames
%     
%     generated_imgs{i} = insertShape(video_imgs{i},'FilledCircle',[corners(:,:,i)' 5*ones(4,1)]);
% end
%
K = 1.0e+02 * [
   7.661088867187500                   0   3.139585628047498
                   0   7.699354248046875   2.503607131410900
                   0                   0   0.010000000000000 ];

pos = cell(num_frames, 1);
rot = cell(num_frames, 1);

% Process all the images
pr = [ 1 0 0;
       0 1 0 ];
tag_width = 0.13;
tag_height = 0.13;
cube_depth = 0.13;
corner_pts = [  tag_width/2,  tag_height/2;
               -tag_width/2,  tag_height/2;
               -tag_width/2, -tag_height/2;
                tag_width/2, -tag_height/2 ];
render_points = [ corner_pts, zeros(4,1);
                  corner_pts, (cube_depth)*ones(4,1) ];
for i=1:num_frames
    % KLT Part
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
    % Find all points in the video frame inside the polygon defined by
    % video_pts
    p = (pr*(K \ [corners(:,:,i)'; ones(1,4)]))';
    H = est_homography(corner_pts,p);
    [proj_pts, pos{i}, rot{i}] = ar_cube(H,render_points,K);
    % Draw a coordinate frame
    frame = 0.05 * [0 0 0; 1 0 0; 0 1 0; 0 0 1];
    frame = bsxfun(@plus,frame * rot{i}',pos{i}') * K';
    frame(:, 1) = frame(:, 1) ./ frame(:, 3);
    frame(:, 2) = frame(:, 2) ./ frame(:, 3);
    frame = frame(:, 1:2);
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'Line',[frame(1,:) frame(2,:)], ...
                        'Color','red','LineWidth',4);
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'Line',[frame(1,:) frame(3,:)], ...
                        'Color','green','LineWidth',4);
    generated_imgs{i} = insertShape(generated_imgs{i}, ...
                        'Line',[frame(1,:) frame(4,:)], ...
                        'Color','blue','LineWidth',4);

    % Copy the RGB values from the logo_img to the video frame
    generated_imgs{i} = draw_ar_cube(proj_pts,generated_imgs{i}); 
    
end

