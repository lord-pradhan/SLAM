%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  %
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
% sig_x = 0.25;
% sig_y = 0.1;
% sig_alpha = 0.1;
% sig_beta = 0.01;
% sig_r = 0.08;
% % 
% sig_x = 0.5;
% sig_y = 0.3;
% sig_alpha = 0.3;
% sig_beta = 0.35;
% sig_r = 1.0;

sig_x = 2;
sig_y = 1;
sig_alpha = 1;
sig_beta = 0.35;
sig_r = 1.0;


%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;

%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

% Write your code here...
k=6;
l = [3,3,7,7,11,11; 
    6,12,8,14,6,12];
euclid =[];
maha = [];

landmark = zeros(2*k, 1);
landmark_cov=[];
for i=1:k
    beta_l = measure(2*(i-1)+1);
    r_l = measure(2*(i-1)+2);
    landmark(2*(i-1)+1 : 2*i , 1) = [pose(1) + r_l*cos(beta_l + pose(3)); 
                                    pose(2) + r_l*sin(beta_l + pose(3))];
    
    J_l = [1,0, -r_l*sin( pose(3) + beta_l ), cos(pose(3) + beta_l), -r_l*sin(pose(3) + beta_l);
            0,1, r_l*cos(pose(3) + beta_l), sin(pose(3) + beta_l), r_l*cos(pose(3) + beta_l)];
    
    cov_l = J_l * diag([diag(pose_cov)', diag(measure_cov)' ]) * J_l';
        
    landmark_cov = blkdiag(landmark_cov, cov_l); % TODO: do this better
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);

    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====

    % Write your code here...
    [x_pre, P_pre] = calcPre(x, P, d, alpha, control_cov);

    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    pause(.01)

    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';

    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====

    % Write your code here...
    for i_l = 1:2:length(arr)
        
        l_x = x(3 + i_l);
        l_y = x(3 + i_l + 1);
        d_rl = (l_x - x(1))^2 + (l_y - x(2))^2;
        
        H_p = [(l_y - x(2))/d_rl, -(l_x - x(1))/d_rl, -1;
                -(l_x - x(1))/sqrt(d_rl), -(l_y - x(2))/sqrt(d_rl), 0];
            
        H_l = [ -(l_y - x(2))/d_rl, (l_x - x(1))/d_rl;
                (l_x - x(1))/sqrt(d_rl), (l_y - x(2))/sqrt(d_rl)];

        K_pos = P_pre(1:3, 1:3) * H_p'/ (H_p * P_pre(1:3, 1:3) * H_p' + measure_cov);
        
        K_l = P_pre(3+i_l:3+i_l+1, 3+i_l:3+i_l+1) * H_l' / ...
                                ( H_l * P_pre(3+i_l:3+i_l+1, 3+i_l:3+i_l+1)*H_l' + measure_cov);
        
        meas_pred = [wrapToPi( atan2(l_y - x(2), l_x - x(1)) - x(3) ); sqrt(d_rl)];
        
        x(1:3) = x_pre(1:3) + K_pos * ( [arr(i_l); arr(i_l+1)] -  meas_pred );
        x(3+i_l:3+i_l+1) = x_pre(3+i_l:3+i_l+1) + K_l * ( [arr(i_l); arr(i_l+1)] -  meas_pred );
        
        P(1:3, 1:3) = (eye(3) - K_pos*H_p) * P_pre(1:3,1:3);
        P(3+i_l:3+i_l+1, 3+i_l:3+i_l+1) = (eye(2) - K_l*H_l) * P_pre(3+i_l:3+i_l+1, 3+i_l:3+i_l+1);
        
%         drawTrajAndMap(x, last_x, P, t);
    end
    %==== Plot ====    
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    pause(.1)
    
    %== Distance stats ==
%     euclid_temp = zeros(1,k); maha_temp = zeros(1,k);
%     for i = 1:k
% %         beta_l = measure(2*(i-1)+1);
% %         r_l = measure(2*(i-1)+2);
%         
%         euclid_temp(1,i) = sqrt( ( x(3+2*(i-1)+1) - l(1,i) )^2 + ( x(3+2*(i-1)+2) - l(2,i) )^2 );
%         maha_temp(1,i) = sqrt( ([x(3+2*(i-1)+1) ;x(3+2*(i-1)+2) ] - l(:,i))' / ...
%                                 P(3+2*(i-1)+1:3+2*(i-1)+2, 3+2*(i-1)+1:3+2*(i-1)+2 ) ...
%                                 * ([x(3+2*(i-1)+1) ;x(3+2*(i-1)+2) ] - l(:,i)) );
%     end
    
%     euclid = [euclid; euclid_temp];
%     maha = [maha; maha_temp];
%     
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);    
end

for i = 1:k
    
    euclid(1,i) = sqrt( ( x(3+2*(i-1)+1) - l(1,i) )^2 + ( x(3+2*(i-1)+2) - l(2,i) )^2 );
    maha(1,i) = sqrt( ([x(3+2*(i-1)+1) ;x(3+2*(i-1)+2) ] - l(:,i))' / ...
                            P(3+2*(i-1)+1:3+2*(i-1)+2, 3+2*(i-1)+1:3+2*(i-1)+2 ) ...
                            * ([x(3+2*(i-1)+1) ;x(3+2*(i-1)+2) ] - l(:,i)) );
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...

hold on
plot(l(1,:), l(2,:), 'k*')

%==== Close data file ====
fclose(fid);
