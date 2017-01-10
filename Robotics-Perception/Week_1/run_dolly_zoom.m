%% load data and set parameters
points = load('points.mat');

d_ref = 4;
f_ref = 400;
pos = 0 : -0.1 : -9.9;

d1_ref = 4;
d2_ref = 20;
H1 = points.points_A(1,2) - points.points_A(2,2);
H2 = points.points_C(1,2) - points.points_C(2,2);
ratio = 2;

%% Dolly Zoom: keep one object's height constant 

f = compute_focal_length(d_ref, f_ref, pos);

for i = 1 : length(f)
    if i == 1
        fprintf('Processing frame %03d / %d...', i, length(f));
    else
        fprintf(repmat('\b',1,12));  
        fprintf('%03d / %d...', i, length(f));
        clf;
    end
    
    figure(1), hold on, axis equal;
    xlim([0,1920]), ylim([0,1080]);
    project_objects(f(i), pos(i), points, 1);
    pause(0.1);
end;
fprintf('\n');

%% Dolly Zoom:  keep one object's height constant and adjust another objects height

[f, pos] = compute_f_pos(d1_ref, d2_ref, H1, H2, ratio, f_ref);

figure(2), hold on, axis equal;
xlim([0,1920]), ylim([0,1080]);
project_objects(f, pos, points, 2);
