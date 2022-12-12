function f = dynamics_acrobot(t, x, u)
m1 = 1 ; m2 = 1;
l1 = 1 ; l2 = 1;g = 9.81;
lc1 = l1/2; lc2 = l2/2;
I1 = 1 ;I2 = 1; 
q1 = x(1,:); q2 = x(2,:); dq1 = x(3,:); dq2 = x(4,:);

m11 = I1 + I2 + m2*l1^2 + 2*m2*l1*lc2.*cos(q2);
m12 = I2 + m2*l1*lc2.*cos(q2);
m22 = I2*ones(1,size(x,2));
M = [m11 m12; m12 m22];

C = [-2*m2*l1*lc2.*sin(q2).*dq2.*dq1-m2*l1*lc2.*sin(q2).*dq2.*dq2;
     m2*l1*lc2.*sin(q2).*dq1.*dq1+zeros(1, size(x,2)).*dq2];

% G is negative for q1 = 0  about vertical
% G is positive for q1 = pi about vertical
G = [m1*g*lc1.*sin(q1) + m2*g.*(l1.*sin(q1)+lc2.*sin(q1+q2));
     m2*g*lc2.*sin(q1+q2)];

dq = [dq1; dq2]; B = [zeros(1,size(x,2)); ones(1,size(x,2))];

ddq = M\(B.*u-C-G);

f = [dq;ddq];
end