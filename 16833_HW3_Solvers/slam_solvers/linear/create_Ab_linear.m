% CREATE_AB_LINEAR
% 16-833 Spring 2020 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first
% pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% prior
J_o = [-1,0,1,0; 0,-1,0,1];
J_l = [-1,0,1,0; 0,-1,0,1];

A(1:p_dim, 1:p_dim) = eye(p_dim)/1e-6;
b(1:p_dim,1) = 0;
% sigmainv_o = inv(sigma_o);
% sigmainv_l = inv(sigma_l);

% add odom jacobian
for i = 1:n_odom
    A( p_dim + o_dim*i-1 : p_dim + o_dim*(i+1)-2, p_dim*i-1 : p_dim*(i+2)-2 ) = sigma_o.^(0.5) \ J_o; 
    b( p_dim + o_dim*i-1 : p_dim + o_dim*(i+1)-2, 1 ) = sigma_o.^(0.5) \ odom(i,:)' ;
end

% add landmark jacobian
odomEnd = p_dim + o_dim*n_odom;
poseEnd = p_dim*n_poses;
for i = 1:n_obs
    i_pose = obs(i, 1);
    i_lm = obs(i,2);
    
    A(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2,...
        p_dim*i_pose-1 : p_dim*(i_pose+1)-2) = sigma_l.^(0.5) \ J_l(:,1:2);
    
    A(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2,...
        poseEnd + l_dim*i_lm-1 : poseEnd + l_dim*(i_lm+1)-2) = sigma_l.^(0.5) \ J_l(:, 3:4);
    
    b(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2, 1) = sigma_l.^(0.5) \ obs(i, 3:4)';
end
%% Make A a sparse matrix 
As = sparse(A);