function im_out=draw_ar_cube(points,im)
%% draw_ar_cube
% Uses projection equation to create images from points
% Inputs:
%     points - size (2 x 8) vectors of projected points to connect
%     im - size (n x m) to render with the image
% Outputs:
%     im_out - size (n x m) with the cube drawn on the image
% Written by Stephen Phillips and Alex Zhu for the Coursera Robotics:Perception
% course
inds = [ 1,2, 1,4, 1,5, 2,3, 2,6, 3,4, 3,7, 4,8, 5,6, 5,8, 6,7, 7,8 ];
X = points(inds,:);
% Compute other positions
for j = 1:2:length(X)
    Xj = X([j,j+1],:);
    im = insertShape(im, 'Line', [round(Xj(1,1:2)), round(Xj(2,1:2)) ], 'LineWidth', 5);
end
im_out = im;




end