clc; clear; close all;

%% run LQR
params.system = 1; % 1 - Acrobot
params.predHorizon = 5;

if(params.system == 1)
    % ode45 needs a row vector for x0 and xd
    xd = [pi 0 0 0];
    x0 = [0.1 0.003 0 0];
    
    x = sym('x',[4;1]); u = sym('u',[1;1]);
    f = @dynamics_acrobot;
    A = double(subs(jacobian(f(0,x,0),x),x,xd')); 
    B = double(subs(jacobian(f(0,xd',u),u),u,0));
    
    Q = diag([10 10 1 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    u = @(x) -K*(x-xd');
    
    tspan = 0:0.01:5;
    [t,x] = ode45(@(t,x)f(t,x,u(x)), tspan, x0);
    
    % animate
    animate_acrobot(t,x)
end