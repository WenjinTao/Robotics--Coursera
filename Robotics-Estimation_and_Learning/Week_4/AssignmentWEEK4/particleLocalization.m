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
M = 10;                        % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles

      
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    

    %   2-2) For each particle, calculate the correlation scores of the particles

    %   2-3) Update the particle weights         
 
    %   2-4) Choose the best particle to update the pose
    
    % 3) Resample if the effective number of particles is smaller than a threshold

    % 4) Visualize the pose on the map as needed
   

end

end



