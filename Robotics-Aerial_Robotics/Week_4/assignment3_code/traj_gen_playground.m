clear; clc;
%% Params

waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';
N = size(waypoints, 2)-1; % Num of polynomial pieces
%% Coeffs matrix of the polynomial
p_c = zeros(7, 8); % Coefficients of the polynomial

p_c(1,:) = ones(1,8); % Coeffs of the poly
 
p = poly2sym(p_c(1,:));

p_d1 = diff(p); % Coeffs of the 1st diff
p_c(2,2:8) = coeffs(p_d1);
p_d2 = diff(p_d1);
p_c(3,3:8) = coeffs(p_d2);
p_d3 = diff(p_d2);
p_c(4,4:8) = coeffs(p_d3);
p_d4 = diff(p_d3);
p_c(5,5:8) = coeffs(p_d4);
p_d5 = diff(p_d4);
p_c(6,6:8) = coeffs(p_d5);
p_d6 = diff(p_d5);
p_c(7,7:8) = coeffs(p_d6); % Coeffs of the 6th diff

% Use head and tail to represent the starting and ending points of each
% piece
head_c = diag(p_c);
head_c = diag(head_c);
head_c = [head_c,zeros(7,1)];

%% A*alpha = b
% Calculate A and b, then alpha = A\b.
% A should be [8N x 8N]; b should be [8N x 1]
A = zeros(8*N);
b = zeros(8*N, 3);
% b(1:N, :) = waypoints(:,1:end-1)';
% b(N+1:2*N, :) = waypoints(:,2:end)';

for i = 1:N
    
    A((i-1)*8+1, (1:8)+(i-1)*8) = head_c(1,:); % Eqn. 19 (N constraints)
    b((i-1)*8+1, :) = waypoints(:,i)';
    
    A((i-1)*8+2, (1:8)+(i-1)*8) = p_c(1,:); % Eqn. 19 (N constraints)
    b((i-1)*8+2, :) = waypoints(:,i+1)';
    
    if i < N
        
        A((i-1)*8+3, (1:16)+(i-1)*8) = [p_c(2,:), -head_c(2,:)]; % Eqn. 21

        A((i-1)*8+4, (1:16)+(i-1)*8) = [p_c(3,:), -head_c(3,:)]; % Eqn. 21

        A((i-1)*8+5, (1:16)+(i-1)*8) = [p_c(4,:), -head_c(4,:)]; % Eqn. 21

        A((i-1)*8+6, (1:16)+(i-1)*8) = [p_c(5,:), -head_c(5,:)]; % Eqn. 21

        A((i-1)*8+7, (1:16)+(i-1)*8) = [p_c(6,:), -head_c(6,:)]; % Eqn. 21

        A((i-1)*8+8, (1:16)+(i-1)*8) = [p_c(7,:), -head_c(7,:)]; % Eqn. 21
    end

end

% Add the other 6 constraints Eqn. 20
A(8*N-5, 1:8) = head_c(2,:);
A(8*N-4, 1:8) = head_c(3,:);
A(8*N-3, 1:8) = head_c(4,:);
A(8*N-2, (1:8)+8*(N-1)) = p_c(2,:);
A(8*N-1, (1:8)+8*(N-1)) = p_c(3,:);
A(8*N  , (1:8)+8*(N-1)) = p_c(4,:);



alpha(:,:,1) = reshape(A\b(:,1), 8, N);
alpha(:,:,2) = reshape(A\b(:,2), 8, N);
alpha(:,:,3) = reshape(A\b(:,3), 8, N);

%% 
d = waypoints(:,2:end) - waypoints(:,1:end-1);
d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
traj_time = [0, cumsum(d0)];
waypoints0 = waypoints;

t = 1.5;

t_index = find(traj_time >= t,1);
if(t_index > 1)
    t = t - traj_time(t_index-1);
end

scale = t/d0(t_index-1);


f_p = squeeze(alpha(:,t_index-1,:))'.*repmat(p_c(1,:),3,1);
f_p = flip(f_p,2);
desired_state.pos = [polyval(f_p(1,:),scale);
                     polyval(f_p(2,:),scale);
                     polyval(f_p(3,:),scale)];

f_v = squeeze(alpha(:,t_index-1,:))'.*repmat(p_c(2,:),3,1);
f_v = flip(f_v(:,2:8),2);
desired_state.vel = [polyval(f_v(1,:),scale);
                     polyval(f_v(2,:),scale);
                     polyval(f_v(3,:),scale)]/d0(t_index-1); 

f_a = squeeze(alpha(:,t_index-1,:))'.*repmat(p_c(3,:),3,1); 
f_a = flip(f_a(:,3:8),2);
desired_state.acc = [polyval(f_a(1,:),scale);
                     polyval(f_a(2,:),scale);
                         polyval(f_a(3,:),scale)]/(d0(t_index-1))^2;
