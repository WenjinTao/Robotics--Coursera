% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);

% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2); % Depend on how many you feed into this function

n_scans = size(scanAngles,1); % Number of scans at certain time

for j = 1:N % for each time,    
      
    % Find grids hit by the rays (in the gird map coordinate)
    x = pose(1,j);
    y = pose(2,j);
    theta = pose(3,j);
    
    d = ranges(:,j); % Num of scans x jth pose
    alpha = scanAngles; % Number of scans x 1
    
    pos_occ = [d.*cos(theta+alpha), -d.*sin(theta+alpha)]' + repmat([x;y],1,n_scans); % 2 x Num of scans
    
    % Find occupied-measurement cells and free-measurement cells
    occ = ceil(myResol*pos_occ) + repmat(myorigin,1,n_scans); % 2 x Num of scans
    
    pos_current = ceil(myResol*[pose(1,j);pose(2,j)]) + myorigin; % Current position of the robot
    
    occ_indices = sub2ind(size(myMap), occ(2,:), occ(1,:)); 
    
    myMap(occ_indices) = myMap(occ_indices) + lo_occ; % Update the occ information of the map
    
    for i = 1:n_scans
        
        [freex, freey] = bresenham(pos_current(1),pos_current(2),occ(1,i),occ(2,i));

        free_indices = sub2ind(size(myMap), freey, freex);
        
        myMap(free_indices) = myMap(free_indices) - lo_free; % Update the free information of the map
        
    end

end

myMap(myMap<lo_min) = lo_min; % Prevent the map from becoming too certain
myMap(myMap>lo_max) = lo_max;

end

