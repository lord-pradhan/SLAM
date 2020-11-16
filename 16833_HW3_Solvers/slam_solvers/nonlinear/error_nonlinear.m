% ERROR_NONLINEAR
% 16-833 Spring 2020 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
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
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_odom - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_landmark - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);    % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize error
err = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% prior
J_o = [-1,0,1,0; 0,-1,0,1];
% J_l = meas_landmark_jacobian();
A(1:p_dim, 1:p_dim) = eye(p_dim)/1e-5;
b(1:p_dim,1) = 0;
% sigmainv_o = inv(sigma_o);
% sigmainv_l = inv(sigma_landmark);

% add odom jacobian
for i = 1:n_odom
    A( p_dim + o_dim*i-1 : p_dim + o_dim*(i+1)-2, p_dim*i-1 : p_dim*(i+2)-2 ) = sigma_odom.^(0.5) \ J_o; 
    b( p_dim + o_dim*i-1 : p_dim + o_dim*(i+1)-2, 1 ) = sigma_odom.^(0.5) \ ...
        ( odom(i,:)' - meas_odom( x(p_dim*i-1, 1), x(p_dim*i, 1), x(p_dim*(i+1)-1, 1), x(p_dim*(i+1), 1)) );
end

% add landmark jacobian
odomEnd = p_dim + o_dim*n_odom;
poseEnd = p_dim*n_poses;
for i = 1:n_obs
    i_pose = obs(i, 1);
    i_lm = obs(i,2);
    
    J_l = meas_landmark_jacobian( x(p_dim*i_pose-1, 1), x(p_dim*i_pose, 1), ...
        x(p_dim*n_poses + l_dim*i_lm - 1, 1), x(p_dim*n_poses + l_dim*i_lm, 1));
    
    A(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2,...
        p_dim*i_pose-1 : p_dim*(i_pose+1)-2) = sigma_landmark.^(0.5) \ J_l(:,1:2);
    
    A(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2,...
        poseEnd + l_dim*i_lm-1 : poseEnd + l_dim*(i_lm+1)-2) = sigma_landmark.^(0.5) \ J_l(:, 3:4);
    
    meas_m = meas_landmark(  x(p_dim*i_pose-1, 1), x(p_dim*i_pose, 1), ...
        x(p_dim*n_poses + l_dim*i_lm - 1, 1), x(p_dim*n_poses + l_dim*i_lm, 1) );
    pred_err = (obs(i, 3:4)');
%     pred_err(1) = wrapToPi(pred_err(1));
    
    b(odomEnd + o_dim*i-1 : odomEnd + o_dim*(i+1)-2, 1) = sigma_landmark.^(0.5) \ pred_err;
end

err = err + ( A*x - b );