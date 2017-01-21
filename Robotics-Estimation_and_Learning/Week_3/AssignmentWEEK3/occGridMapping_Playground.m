% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
% function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
log_odd = myMap;
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2); % Depend on how many you feed into this function

n_scans = size(scanAngles,1);


for j = 1:500 % for each time,
    
      
    % Find grids hit by the rays (in the gird map coordinate)
    x = pose(1,j);
    y = pose(2,j);
    theta = pose(3,j);
    
    d = ranges(:,j); % Num of scans x jth pose
    alpha = scanAngles; % Number of scans x 1
    
    pos_occ = [d.*cos(theta+alpha), -d.*sin(theta+alpha)]' + repmat([x;y],1,n_scans); % 2 x Num of scans
    
    % Find occupied-measurement cells and free-measurement cells
    occ = ceil(myResol*pos_occ) + repmat(myorigin,1,n_scans); % 2 x Num of scans
    
    occ_indices = sub2ind(size(myMap), occ(2,:), occ(1,:));
    log_odd(occ_indices) = log_odd(occ_indices) + lo_occ;
    
    % get cells in between
%     free = [];

    for i = 1:n_scans
        
        [freex, freey] = bresenham(myorigin(1),myorigin(2),occ(1,i),occ(2,i));

        free_indices = sub2ind(size(myMap), freey, freex);
        
        log_odd(free_indices) = log_odd(free_indices) - lo_free;
        
    
    end

    % Update the log-odds
    % Case I: cell with z=1
    % log odd <- 0+log(odd_occ)
    % Case II: cell with z=0
    % log odd <- 0-log(odd_free)
    

    
%     log_odd(free) = log_odd(free) - lo_free;  

    % Saturate the log-odd values
    

    % Visualize the map as needed
   

end
log_odd(log_odd<lo_min) = lo_min;
log_odd(log_odd>lo_max) = lo_max;
myMap = log_odd;

% end
figure,
imagesc(myMap); 
colormap('gray'); axis equal;