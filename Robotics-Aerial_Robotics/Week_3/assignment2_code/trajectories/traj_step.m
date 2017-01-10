function desired_state = traj_step(t, ~)

acc = zeros(2,1);
vel = zeros(2,1);
if t < 1
  pos = zeros(2,1);
else
  pos = [0.5; 0.0];
end

desired_state.pos = pos;
desired_state.vel = vel;
desired_state.acc = acc;

end
