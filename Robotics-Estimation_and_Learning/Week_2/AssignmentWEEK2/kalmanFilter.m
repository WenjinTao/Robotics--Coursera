function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    
    % Dynamic model/ system model:
    % s = [px py vx vy]'
    
    dt = t - previous_t;
    
    % Transition matrix A:
    A = eye(4);
    A(1,3) = dt;
    A(2,4) = dt;
    
    C = [1 0 0 0;
         0 1 0 0];
     
    % Model noise
    sigma_m = 1e3*eye(4);
    
    % Measurement noise
    sigma_o = 0.01*eye(2);
    
    P = A*param.P*A' + sigma_m;
    
    R = sigma_o;   
    
    K = P*C'*(R + C*P*C')^(-1);
    
    % Updata the state
    state = A*state' + K*([x;y] - C*A*state');
    
    % Must update the covariance of the state
    param.P = P - K*C*P;

        
    % Predict 330ms into the future
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
    
    % Reshape the state to 1x4
    state = state';
end
