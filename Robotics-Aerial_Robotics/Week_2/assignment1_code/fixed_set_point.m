function [ s_des ] = fixed_set_point(t, z_des)
%FIXED_SET_POINT  Outputs a constant desired state = [z_des;0] except at t = 0 where it returns [0;0]

if t == 0
  s_des = [0;0];
else
  s_des = [z_des;0];
end

end
