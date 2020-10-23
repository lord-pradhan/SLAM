function [x_pre, P_pre] = calcPre(x, P, d, alpha, control_cov)

x_pre=x;
x_pre(1) = x(1) + cos(x(3))*d;
x_pre(2) = x(2) + sin(x(3))*d;
x_pre(3) = x(3) + alpha;

P_pre = P;

P_w = blkdiag(P(1:3, 1:3), control_cov);

A_jac= [1, 0, -sin(x(3))*d, cos(x(3)), -sin(x(3)), 0;
            0, 1, cos(x(3))*d, sin(x(3)), cos(x(3)), 0;
            0, 0, 1, 0, 0, 1];

P_pre(1:3, 1:3) = A_jac * P_w * A_jac';

end

