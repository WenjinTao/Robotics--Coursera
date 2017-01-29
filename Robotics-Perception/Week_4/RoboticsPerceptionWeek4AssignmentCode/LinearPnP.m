function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

[N, ~] = size(X);
x = [x ones(N,1)];
% Calibrate the x values
x = K\x';
x = x';
X = [X ones(N,1)];
A = zeros(3*N, 12);

for i = 1:N
    
    A(i*3-2, 5:8) = -X(i,:);
    A(i*3-2, 9:12) = x(i,2)*X(i,:);
    A(i*3-1, 1:4) = X(i,:);
    A(i*3-1, 9:12) = -x(i,1)*X(i,:);
    A(i*3, 1:4) = -x(i,2)*X(i,:);
    A(i*3, 5:8) = x(i,1)*X(i,:);
        
end

[~, ~, V] = svd(A);

P = reshape(V(:,end), 4, 3)';

P = P/P(end);

% P_c = K\P;
R = P(:,1:3);
t = P(:,4);

[u, d, v] = svd(R);

if det(u*v') > 0
    R = u*v';
    t = t/d(1);
else
    R = -u*v';
    t = -t/d(1);
end

C = -R'*t;

end





