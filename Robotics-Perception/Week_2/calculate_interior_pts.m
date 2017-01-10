function [ interior_pts ] = calculate_interior_pts( image_size, corners )
% calculate_interior_pts takes in the size of an image and a set of corners
% of a shape inside that image, and returns all (x,y) points in that image
% within the corners
% Written for the University of Pennsylvania's Robotics:Perception course

% YOU SHOULDN'T NEED TO CHANGE THIS
[X, Y] = meshgrid(1:image_size(2), 1:image_size(1));
X=X(:);
Y=Y(:);

interior_inds = inpolygon(X,Y,corners(:,1), corners(:,2));
interior_pts = [X(interior_inds),...
    Y(interior_inds)];
end

