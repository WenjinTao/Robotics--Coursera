% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose; % 3 x 1
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 400;                        % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
% P = repmat(myPose(:,1), [1, M]); % 3 x M

n_scans = size(scanAngles,1);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles
    P = repmat(myPose(:,j-1), [1, M]); % 3 x M
    
    sigma_m = 0.01*randn(3, M); % 3 x M
    P = P + sigma_m;
    weight_P = ones(1, M)/M; % 1 x M
      
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)  
    %   2-2) For each particle, calculate the correlation scores of the particles
    %   2-3) Update the particle weights          
    %   2-4) Choose the best particle to update the pose    
    % 3) Resample if the effective number of particles is smaller than a threshold
    % 4) Visualize the pose on the map as needed    
        
    d = ranges(:,j);
    alpha = scanAngles; % n_scans x 1
    
    for i = 1:M
        % A particle is an array of a position and orientation hypothesis (x, y, theta) 
        x = P(1,i);
        y = P(2,i);
        theta = P(3,i);
        
        pos_occ = [d.*cos(theta+alpha), -d.*sin(theta+alpha)]' + repmat([x;y],1,n_scans); % 2 x Num of scans    
        lidar_occ = ceil(myResolution*pos_occ) + repmat(myOrigin,1,n_scans); % 2 x Num of scans
        

        if sum(lidar_occ(2,:)>size(map,1)) || sum(lidar_occ(1,:)>size(map,2)) || sum(lidar_occ(1,:)<=0)|| sum(lidar_occ(2,:)<=0)   
             score(i) = 0;
        else
            lidar_occ_indices = sub2ind(size(map), lidar_occ(2,:), lidar_occ(1,:));

            score(i) = sum(map(lidar_occ_indices)>0.49);
            score(i) = score(i) - sum(map(lidar_occ_indices)<0.49)*.5;
        end
    end
    
    weight_P = weight_P.*score; % 1 x M
    weight_P = weight_P/sum(weight_P); % Normalization
    
%     myPose(:,j) = sum(repmat(weight_P,3,1).*P);
    [~, idx] = max(score);
    myPose(:,j) = P(:,idx);
    
    % Resampling step
    C = 200; % Threshold
    n_effective = sum(weight_P)^2/sum(weight_P.^2);
    if n_effective < C
        j = j - 1;  
    end   

end

figure;
imagesc(map); hold on;
colormap('gray');
axis equal;
hold on;
plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');

end



