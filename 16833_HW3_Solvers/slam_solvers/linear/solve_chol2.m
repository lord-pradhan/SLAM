% SOLVE_CHOL2
% 16-833 Spring 2020 - *Stub* Provided
% Solves linear system using second Cholesky method
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using the specified
%             version of the Cholesky decomposition
%     R     - R factor from the Cholesky decomposition
%
function [x, R] = solve_chol2(A, b)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[R, flag, P] = chol(A'*A);
y = back_sub(R', A'*b);
x = forward_sub(R,y);
end