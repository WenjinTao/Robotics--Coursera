function [ f ] = compute_focal_length( d_ref, f_ref, pos )
%% Compute camera focal length with given camera position.  
% 
% In this function, multiple camera positions will be given. The camera is
% placed on z axis and thus only z axis for the camera position is given.
% We'll need to compute corresponding focal length to achieve Dolly Zoom
% effect for each camera position.
%
% Input:
% - d_ref: 1 by 1 double, distance of the object whose size remains constant
% - f_ref: 1 by 1 double, previous camera focal length
% - pos: 1 by n, each element represents camera center position on the z axis.
% Output:
% - f: 1 by n, camera focal length, each element corresponds to an element
% in pos, i.e., f(i) is the focal length for Dolly Zoom effect when the camera is placed at pos(i)

% YOUR CODE HERE

f = f_ref * (d_ref-pos) / d_ref;

% f = f_ref/d_ref * (d_ref-pos);

end

