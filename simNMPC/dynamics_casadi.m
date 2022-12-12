function [f,x,u] = dynamics_casadi(system)
import casadi.*

if(system==1)
    % 1 - acrobot
    % states
    q = SX.sym('q',2);
    dq = SX.sym('dq',2);
    x = [q;dq];
    
    % controls
    u = SX.sym('tau',1);

    % dynamics
    f = dynamics_acrobot(0,x,u);
end

dxdt = f;
f = Function('f',{x,u},{dxdt}); % nonlinear mapping function f(x,u)

end
