function project_objects( f, pos, points, fid )
% render synthetic image using given camera focal length and camera
% position
%
% Input:
% - f: double camera focal length
% - pos: double represent camera center position in z axis.
% - points: 3D coordinates for vetice on polygons (use "load points.mat" to get)
% Output:
% - img: 1080*1920*3 matrix, the output render image
%
% after runing Dolly_Zoom
% you can use 'imwrite(img, 'output.png');' to save the image.

    color_1 = [0 1 0];
    color_2 = [1 0 1];
    color_3 = [0 0 1];
    
    points_A = points.points_A;
    points_B = points.points_B;
    points_C = points.points_C;
    
    figure(fid);
    % object 3
    p2d = project(points_C, f, pos);
    color = color_3;
    fill(p2d([1,2,4,3],1), p2d([1,2,4,3],2), color);
    fill(p2d([3,4,6,5],1), p2d([3,4,6,5],2), color);
    
    % object 1
    p2d = project(points_A, f, pos);
    color = color_1;
    fill(p2d([1,2,4,3],1), p2d([1,2,4,3],2), color);
    fill(p2d([3,4,6,5],1), p2d([3,4,6,5],2), color);

    % object 2
    p2d = project(points_B, f, pos);
    color = color_2;
    fill(p2d([1,2,3],1), p2d([1,2,3],2), color);
    fill(p2d([2,3,4],1), p2d([2,3,4],2), color);
end

function [ p2d ] = project( p3d, f, pos )
% use for compute vertex image position from given vertex 3D position and
% camera focal length and camera position

% Input:
% - p3d: n by 3, 3D vertex position in world coordinate system
% - f: double, camera focal length
% - pos : double, camera center position
% Output:
% - p2d: n by 2, each row represents vertex image position, in pixel unit

p2d(:,1) = p3d(:,1)*f./(p3d(:,3) - pos) + 960;
p2d(:,2) = p3d(:,2)*f./(p3d(:,3) - pos) + 540;

end