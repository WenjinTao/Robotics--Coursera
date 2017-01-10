function quad_state = simStateToQuadState(sim_state)
% simStateToQuadState Convert sim state to the 13 element quad state
% Sim state vector =  [y; z; phi; y_dot; z_dot; phi_dot]
% Quad state vector = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

quad_state = zeros(13,1);
quad_state(1) = 0;
quad_state(2) = sim_state(1);
quad_state(3) = sim_state(2);
quad_state(4) = 0;
quad_state(5) = sim_state(4);
quad_state(6) = sim_state(5);
quad_state(7) = cos(-sim_state(3)/2);
quad_state(8) = sin(-sim_state(3)/2);
quad_state(9) = 0;
quad_state(10) = 0;
quad_state(11) = 0;
quad_state(12) = sim_state(6);
quad_state(13) = 0;

end
