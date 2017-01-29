function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

[N,~] = size(X0);
X = zeros(N,3);

for j = 1:10
%     X_old = X;
    for i =1:N
        X_refine = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)');   
        X(i,:) = X_refine';
    end
%     e = sum(sum((X-X_old).^2));
%     if e < 0.5
%         break;
%     end
%     
end 

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
% X0 shape (3x1)
J = [Jacobian_Triangulation(C1, R1, K, X0);
     Jacobian_Triangulation(C2, R2, K, X0);
     Jacobian_Triangulation(C3, R3, K, X0)];
 

uvw_1 = K*R1*(X0-C1); % Eqn.(4)
uvw_2 = K*R2*(X0-C2);
uvw_3 = K*R3*(X0-C3);
u_1 = uvw_1(1); v_1 = uvw_1(2); w_1 = uvw_1(3);
u_2 = uvw_2(1); v_2 = uvw_2(2); w_2 = uvw_2(3);
u_3 = uvw_3(1); v_3 = uvw_3(2); w_3 = uvw_3(3);

b = [x1, x2, x3]';
f = [u_1/w_1, v_1/w_1, u_2/w_2, v_2/w_2, u_3/w_3, v_3/w_3]'; % Eqn.(7)

% Update X_new = X_old + delta_x
delta_X = (J'*J)\J'*(b-f);
X = X0 + delta_X;

end

function J = Jacobian_Triangulation(C, R, K, X)
% X shape (3x1)

uvw = K*R*(X-C); % Eqn.(4)
u = uvw(1);
v = uvw(2);
w = uvw(3);
f = K(1);
p_x = K(1,3);
p_y = K(2,3);

d_u = [f*R(1,1)+p_x*R(3,1), f*R(1,2)+p_x*R(3,2), f*R(1,3)+p_x*R(3,3)];
d_v = [f*R(2,1)+p_y*R(3,1), f*R(2,2)+p_y*R(3,2), f*R(2,3)+p_y*R(3,3)];
d_w = R(3,:);

d_f1 = [(w*d_u-u*d_w)/w^2;
        (w*d_v-v*d_w)/w^2];
J = [d_f1']';

end
