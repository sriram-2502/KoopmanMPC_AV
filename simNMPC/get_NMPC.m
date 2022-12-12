function [zval, pred_traj] = get_NMPC(params,args)
import casadi.*
%% MPC setup
dT = params.dTsim; % sampling time [s]
N = params.predHorizon; % prediction horizon

%% get system dynamics
system = params.system; %1 - acrobot
[f,x,u] = dynamics_casadi(system);
n_states = length(x); 
n_controls = length(u);

%% build augmented state matrix
X = SX.sym('X',n_states,N+1);
U = SX.sym('U',n_controls,N);

% parameters (which include the initial and the reference state of the robot)
p = SX.sym('p',n_states + n_states);

if(args.single_shoot)
    X(:,1) = p(1:n_states); % initial state
    for k = 1:N
        x_i = X(:,k);  u_i = U(:,k);
        % RK4 to get next state predictions
        k1 = f(x_i,         u_i);
        k2 = f(x_i+dT/2*k1, u_i);
        k3 = f(x_i+dT/2*k2, u_i);
        k4 = f(x_i+dT*k3,   u_i);
        X(:,k+1) = x_i + dT/6*(k1+2*k2+2*k3+k4);
    end
    pred_traj = Function('optim_traj',{U,p},{X});

else
    x_ini = X(:,1);
    g = [];

    % initial condition constraints (equality)
    g = [g; x_ini - p(1:n_states)]; 

    % dynamics constraints (equality)
    for k = 1:N
        x_i = X(:,k);  u_i = U(:,k);    
        % RK4 to get next state predictions
        k1 = f(x_i,         u_i);
        k2 = f(x_i+dT/2*k1, u_i);
        k3 = f(x_i+dT/2*k2, u_i);
        k4 = f(x_i+dT*k3,   u_i);
        x_next = x_i + dT/6*(k1+2*k2+2*k3+k4);
        g = [g; X(:,k+1) - x_next]; 
    end

    % Equality constraints
    args.lbg = 0; 
    args.ubg = 0;
end

%% compute augmented cost
% weights Q and R for obj
if(args.single_shoot)
    Q = diag([1e2 1e2 1e2 1e2]);
    P = diag([1e6 1e6 1e6 1e6]);
    R = 1e-6*eye(n_controls);
else
    Q = diag([1e2 1e2 1e2 1e2]);
    P = diag([1e6 1e6 1e6 1e6]);
    R = 1e-6*eye(n_controls);
end

% compute cost
obj = 0;
x_ref = p(n_states+1:end);
for k=1:N-1
    x_i = X(:,k);  u_i = U(:,k); 
    obj = obj + (x_i-x_ref)'*Q*(x_i-x_ref) + u_i'*R*u_i; % calculate obj
end
% terminal cost
x_n = X(:,N);
obj = obj + (x_n-x_ref)'*P*(x_n-x_ref);

%% set up optimization problem
if(args.single_shoot)
    opt_var = U(:);
    nlp_prob = struct('f', obj, 'x', opt_var, 'p', p);
else
    opt_var = [X(:); U(:)];
    nlp_prob = struct('f', obj, 'x', opt_var, 'g', g, 'p', p);
end

% set up solver options
opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0; %0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

% get solution
if(args.single_shoot)
    args.x0 = 1e-2*ones(size(opt_var)); % tunable
    sol = solver('x0',args.x0,'p', args.p);
    u = full(sol.x);
    zval = u(1:n_controls);
else
    args.x0 = 1e-2*ones(size(opt_var)); % tunable
    sol = solver('x0',args.x0, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
    u = full(sol.x(n_states*(N+1)+1:end));
    zval = u(1:n_controls);
    pred_traj = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
end

full(sol.f)

end