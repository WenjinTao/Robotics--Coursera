function desired_state = traj_line(t, ~)

initial_pos = [0; 1];
v_max = 2;
a_max = 2;

if t <= v_max/a_max
  dt = t;
  acc = [a_max; 0];
  vel = acc*dt;
  pos = 0.5*acc*dt.^2;
elseif t <= 2*v_max/a_max
  dt = t - v_max/a_max;
  acc = [0; 0];
  vel = [v_max; 0];
  pos = [v_max.^2/(2*a_max);0] + [v_max*dt; 0];
elseif t <= 3*v_max/a_max
  dt = t - 2*v_max/a_max;
  acc = [-a_max; 0];
  vel = [v_max; 0] + acc*dt;
  pos = [3*v_max.^2/(2*a_max); 0] + [v_max;0]*dt + 0.5*acc*dt.^2;
else
  acc = [0;0];
  vel = [0;0];
  pos = [2*v_max.^2/a_max; 0];
end

desired_state.pos = initial_pos + pos;
desired_state.vel = vel;
desired_state.acc = acc;

end
