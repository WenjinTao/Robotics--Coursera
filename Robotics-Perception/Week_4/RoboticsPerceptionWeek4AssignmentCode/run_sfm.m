clear all;  close all;  clc;

% Load SIFT keypoints for three images
load('data.mat')
x1 = data.x1;
x2 = data.x2;
x3 = data.x3;

% Load camera calibration parameters
K = data.K;

% Estimate fundamental matrix
F = EstimateFundamentalMatrix(x1, x2);

% Estimate essential matrix from fundamental matrix
E = EssentialMatrixFromFundamentalMatrix(F,K);

C = data.C;
R = data.R;

% Obtain 3d points using correct camera pose
X = LinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2);

% Find the third camera pose using Linear PnP
[C3, R3] = LinearPnP(X, x3, K);


% Calculate reprojection points
x1p = K * X';
x1p = x1p ./ repmat(x1p(3, :), [3, 1]);
x2p = K * R * (X' - repmat(C, [1 size(X,1)]));
x2p = x2p ./ repmat(x2p(3, :), [3, 1]);
x3p = K * R3 * (X' - repmat(C3, [1 size(X,1)]));
x3p = x3p ./ repmat(x3p(3, :), [3, 1]);

% Display correspondence points between SIFT keypoints and reprojection
DisplayCorrespondence(data.img1, x1, x1p(1:2,:)');
DisplayCorrespondence(data.img2, x2, x2p(1:2,:)');
DisplayCorrespondence(data.img3, x3, x3p(1:2,:)');

% Nonlinear triangulation
X = Nonlinear_Triangulation(K, zeros(3,1), eye(3), C, R, C3, R3, x1, x2, x3, X);

% Display point cloud and 3 camera poses
Display3D({zeros(3,1), C, C3}, {eye(3), R, R3}, X);

% Calculate reprojection points
x1p = K * X';
x1p = x1p ./ repmat(x1p(3, :), [3, 1]);
x2p = K * R * (X' - repmat(C, [1 size(X,1)]));
x2p = x2p ./ repmat(x2p(3, :), [3, 1]);
x3p = K * R3 * (X' - repmat(C3, [1 size(X,1)]));
x3p = x3p ./ repmat(x3p(3, :), [3, 1]);

% Display correspondence points between SIFT keypoints and reprojection
DisplayCorrespondence(data.img1, x1, x1p(1:2,:)');
DisplayCorrespondence(data.img2, x2, x2p(1:2,:)');
DisplayCorrespondence(data.img3, x3, x3p(1:2,:)');
