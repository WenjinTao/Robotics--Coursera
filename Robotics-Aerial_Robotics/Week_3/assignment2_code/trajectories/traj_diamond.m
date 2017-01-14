function desired_state = traj_diamond(t, cur_state)

v_max = 2;
a_max = 4;
initial_pos1 = [0; 1.8];
initial_pos2 = initial_pos1 + [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)]*[2*v_max.^2/a_max; 0];
initial_pos3 = initial_pos2 + [cos(-pi/4) -sin(-pi/4); sin(-pi/4) cos(-pi/4)]*[2*v_max.^2/a_max; 0];
initial_pos4 = initial_pos3 + [cos(-3*pi/4) -sin(-3*pi/4); sin(-3*pi/4) cos(-3*pi/4)]*[2*v_max.^2/a_max; 0];

t_seg = 4*v_max/a_max;
if(t >= 4*t_seg)
    t = 4*t_seg;
    t_prune = t_seg;
else
    t_prune = mod(max(0,t-0.00001), t_seg);
end

if t <= t_seg
    initial_pos = initial_pos1;    
elseif t <= 2*t_seg 
    initial_pos = initial_pos2;
elseif t <= 3*t_seg 
    initial_pos = initial_pos3;
else
    initial_pos = initial_pos4;
end

theta = pi/4 - floor(max(0,t-0.00001)/t_seg)*pi/2;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

if t_prune <= v_max/a_max
  dt = t_prune;
  acc = [a_max; 0];
  vel = acc*dt;
  pos = 0.5*acc*dt.^2;
elseif t_prune <= 2*v_max/a_max
  dt = t_prune - v_max/a_max;
  acc = [0; 0];
  vel = [v_max; 0];
  pos = [v_max.^2/(2*a_max);0] + [v_max*dt; 0];
elseif t_prune <= 3*v_max/a_max
  dt = t_prune - 2*v_max/a_max;
  acc = [-a_max; 0];
  vel = [v_max; 0] + acc*dt;
  pos = [3*v_max.^2/(2*a_max); 0] + [v_max;0]*dt + 0.5*acc*dt.^2;
else
  acc = [0;0];
  vel = [0;0];
  pos = [2*v_max.^2/a_max; 0];
end

desired_state.pos = initial_pos + R*pos;
desired_state.vel = R*vel;
desired_state.acc = R*acc;

end
