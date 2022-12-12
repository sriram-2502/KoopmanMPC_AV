clc; clear; close all;
addpath acrobot
%% get params
params.dTsim = 0.2;
params.predHorizon = 5;
params.system = 1; % 1 - acrobot
params.Tmpc = 0.05; % MPC freq

% simulation time
SimTimeDuration = 10;  % [sec]
MAX_ITER = floor(SimTimeDuration/params.dTsim);

% initialize
tstart = 0;
tend = params.dTsim;
xd = [pi 0 0 0]';
x0 = [0 0.003 0 0]';
[tout,Xout,Uout,X_pred] = deal([]);
xt = x0;

% specify constraints (To do)
args = [];

% specify single or multi shooting
args.single_shoot = 1; % 1 - single shooting, 0 - multi shooting

%% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
for ii = 1:MAX_ITER
    % calculate time
    t_ = params.dTsim * (ii-1) + params.Tmpc * (0:params.predHorizon-1);
    
    % set initial and ref states for each iteration
    args.p = [xt;xd]; 
    
    % do MPC
    [zval, pred_traj] = get_NMPC(params,args);
    ut = zval;
    
    % predictions
    if(args.single_shoot)
        x_pred = pred_traj(ut,args.p);
    else
        x_pred = pred_traj;
    end
  
    % simulte the system
    [t,X] = ode45(@(t,x)dynamics_acrobot(t,x,ut),[tstart,tend],xt);
    
    % update initial condition and time
    xt = X(end,:)';
    tstart = tend;
    tend = tstart + params.dTsim;

    % log states and control 
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(ut',[lent,1])];
    X_pred = [X_pred;repmat(full(x_pred)', [lent,params.predHorizon])];

    waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
end
close(h_waitbar)
fprintf('Calculation Complete!\n')
toc

%% animate
if(params.system == 1)
    animate_acrobot(tout,Xout,X_pred,params)
end