function [ projected_img ] = inverse_warping( img_final, img_initial, pts_final, pts_initial )
% inverse_warping takes two images and a set of correspondences between
% them, and warps all the pts_initial in img_initial to the pts_final in 
% img_final
% Written for the University of Pennsylvania's Robotics:Perception course

% YOU SHOULDN'T NEED TO CHANGE THIS
pts_final = ceil(pts_final);
pts_initial = ceil(pts_initial);

ind_final= sub2ind([size(img_final,1), size(img_final,2)],...
    pts_final(:,2),...
    pts_final(:,1));
ind_initial = sub2ind([size(img_initial,1) size(img_initial,2)],...
    pts_initial(:,2),...
    pts_initial(:,1));

projected_img = img_final;

for color = 1:3
    sub_img_final = img_final(:,:,color);
    sub_img_initial = img_initial(:,:,color);
    sub_img_final(ind_final) = sub_img_initial(ind_initial)*0.5 + sub_img_final(ind_final)*0.5;
    projected_img(:,:,color) = sub_img_final;
end

end

