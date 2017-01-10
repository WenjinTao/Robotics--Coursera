function desired_state = traj_sine(t, ~)

initial_pos = [0; 0];
v_max_y = 2;
a_max_y = 2;

% Y
if t <= v_max_y/a_max_y
  dt = t;
  acc_y = a_max_y;
  vel_y = acc_y*dt;
  pos_y = 0.5*acc_y*dt.^2;
elseif t <= 2*v_max_y/a_max_y
  dt = t - v_max_y/a_max_y;
  acc_y = 0;
  vel_y = v_max_y;
  pos_y = v_max_y.^2/(2*a_max_y) + v_max_y*dt;
elseif t <= 3*v_max_y/a_max_y
  dt = t - 2*v_max_y/a_max_y;
  acc_y = -a_max_y;
  vel_y = v_max_y + acc_y*dt;
  pos_y = 3*v_max_y.^2/(2*a_max_y) + v_max_y*dt + 0.5*acc_y*dt.^2;
else
  acc_y = 0;
  vel_y = 0;
  pos_y = 2*v_max_y.^2/a_max_y;
end

% Z
t_max = 3*v_max_y/a_max_y;
omega = 4*pi/t_max;
if t < t_max
  pos_z = 0.25*(1 - cos(omega*t));
  vel_z = 0.25*omega*sin(omega*t);
  acc_z = 0.25*(omega.^2)*cos(omega*t);
else
  pos_z = 0.25*(1 - cos(omega*t_max));
  vel_z = 0;
  acc_z = 0;
end

desired_state.pos = initial_pos + [pos_y; pos_z];
desired_state.vel = [vel_y; vel_z];
desired_state.acc = [acc_y; acc_z];

end
