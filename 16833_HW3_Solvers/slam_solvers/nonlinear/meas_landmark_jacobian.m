% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2020 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d = sqrt((lx-rx)^2 + (ly-ry)^2);
H = [ (ly-ry)/d^2 , -(lx-rx)/d^2, -(ly-ry)/d^2 , (lx-rx)/d^2;
        -(lx-rx)/d, -(ly-ry)/d, (lx-rx)/d, (ly-ry)/d];