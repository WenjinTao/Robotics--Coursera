function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography

% Enforce the z component of t is positive and multiply H by -1 if necessary
if H(3,3)<0
    H = -H;
end

h1 = H(:,1);
h2 = H(:,2);
h3 = H(:,3);

t = h3/norm(h1);

[U, ~, V] = svd([h1, h2, cross(h1, h2)]); % Sigular Value Decomposition,SVD

R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

% YOUR CODE HERE: Project the points using the pose
X = render_points;
n = size(X, 1);

% [Nx2] = [3x3]([3x3]*[Nx3]' + [3xN])
proj_points = (K*(R*X' + repmat(t, 1, n)))';

proj_points = proj_points./repmat(proj_points(:,3), 1, 3);

proj_points = proj_points(:,1:2);

end
