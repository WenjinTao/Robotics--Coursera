function [ params ] = sys_params()
% sys_params System parameters

params.gravity = 9.81;
params.mass = 0.18;
params.Ixx = 0.00025;
params.arm_length = 0.086;

params.minF = 0;
params.maxF = 2*params.mass*params.gravity;

end
