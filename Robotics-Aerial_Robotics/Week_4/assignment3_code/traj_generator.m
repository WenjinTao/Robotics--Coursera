function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% nargin
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

persistent waypoints0 traj_time d0 alpha p_c
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    N = size(waypoints, 2)-1; % Num of polynomial pieces
    
    % Coeffs matrix of the polynomial
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

    % A*alpha = b
    % Calculate A and b, then alpha = A\b.
    % A should be [8N x 8N]; b should be [8N x 1]
    A = zeros(8*N);
    
    b = zeros(8*N, 3);
%     b(1:N, :) = waypoints(:,1:end-1)';
%     b(N+1:2*N, :) = waypoints(:,2:end)';
    

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
    
    
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
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

    end

    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end

