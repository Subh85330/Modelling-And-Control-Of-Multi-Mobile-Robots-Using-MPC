% Create a .mat file with all fixed simulation parameters to solve
% the optimization, as well as the tuning parameters

% Clear everything
clear;


%% Simulation Parameters


% Model parameters for the quad + controller system
% It's model as a discrete second order system
model_params.zeta_xy = 0.6502;
model_params.tau_xy = 0.3815;
model_params.omega_xy = 1/model_params.tau_xy;
% model_params.zeta_z = 0.9103;
% model_params.tau_z = 0.3; %time period
% model_params.omega_z = 1/model_params.tau_z;

% VICON Measurements noise std for position and velocity data
std_p = 0.00228682;
std_v = 0.0109302;

% Dimension of space - 3 = 3D, 2 = 2D
ndim = 2; 

% Time settings and variables
T = 30;          % simulation duration
h = 0.2;         % time step between MPC updates
tk = 0:h:T;      % coarse discrete time base
K = T/h + 1;     % number of time steps to simulate
Ts = 0.01;       % send h/Ts cmds in-between MPC updates 
t = 0:Ts:T;      % interpolated time vector
k_hor = 16;      % horizon length - duration of (k_hor-1)*h sec
T_segment = 1.0; % fixed time length of each Bezier segment

% Collision ellipsoid parameters
order_a = 2;         % order of the ellipsoid - choose between 2 and 4
rmin_a = 0.3;       % X-Y protection radius for collisions
c_a = [1.0, 1.0];           
E_a = diag(c_a); % scaling vector to compute distances to ellipsoid 
E1_a = E_a^(-1);
E2_a = E_a^(-order_a);

% Bezier curve parameters. Note that d > deg_poly always
deg_poly = 2;  % degree of differentiability required for the position
l = 3;         % number of Bezier curves to concatenate
d = 5;         % degree of the bezier curve

% Physical limits of the robot - position and acceleration bounds
% phys_limits.pmin = [-0.7937,-0.7937,0.2];
% phys_limits.pmax = [0.7937,0.7937,1.7874];
phys_limits.pmin = [-1.5, -1.5];
phys_limits.pmax = [10, 10];
phys_limits.amax = 0.5;
phys_limits.amin = -0.5;



%% MPC Parameters
% Create a .mat file with the MPC tuning parameters

%%%%%%%%%% Energy minimization cost %%%%%%%%%%%%%%%%%%
cost_acc = .008;

%%%%%%%%%% Goal tracking error cost %%%%%%%%%%%%%%%%%% 
% s refers to cost and spd refers to the amount of time steps 
% to include in the error minimization
% Case 1: no collisions in the horizon - go fast
s_free = 100;
spd_f = 3;

% Case 2: collisions in the horizon - go slower
s_obs = 100;
spd_o = 1;

% Case 3: colliding scenario, change setpoint and repel rapidly
s_repel = 1000;
spd_r = 10;

%%%%%%%%%% Collision relaxation penalty %%%%%%%%%%%%
lin_coll_penalty = -1e4;
quad_coll_penalty = 1;

%%%%%%%%%% Tolerances to trigger a replan %%%%%%%%%%%%
err_tol_pos = 0.05;         % tolerance between predicted and sensed pos
err_tol_vel = 0.5;          % tolerance between predicted and sensed vel
max_cost = 0.8*ones(ndim, 1);   % max threshold on the strain cost function
min_cost = -0.1*ones(ndim, 1); % min threshold on the strain cost function



%%
save('input_params.mat')
