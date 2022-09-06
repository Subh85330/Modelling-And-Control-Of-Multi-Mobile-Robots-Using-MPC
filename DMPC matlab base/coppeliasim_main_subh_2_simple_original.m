clc
clear all
close all
warning('off','all')
rosinit('192.168.1.9')

% Load simulation parameters
load('config/input_params.mat')

% Choose what data to visualize
visualize = 1;      % 3D visualization of trajectory and predictions
view_states = 0;    % pos, vel and acc of all agents
view_distance = 0;  % inter-agent distance over time
view_cost = 0;      % value of the replanning cost function
global debug_constr;
debug_constr = 0;


% Disturbance applied to the model within a time frame
disturbance =   0;       % activate the disturbance
agent_disturb = [1];   % choose which agents to perturb
disturbance_k = [10:20];  % timesteps to apply the perturbation

% We will assume that all the rogue agents are labelled after the commanded agents

% Number of vehicles in the problem
N = 4;
N_obs = 1;

% OBSTACLE DEFINITIONS
% first obstacle
rmin_r(1) = 1;
c_r(1, :) =  [0.3, 3];

% second obstacle
rmin_r(2) = 1.0;
c_r(2, :) =  [0.1, 0.1];

% third obstacle
rmin_r(3) = 1.0;
c_r(3, :) =  [0.4, 0.5];

% fourth obstacle
rmin_r(4) = 1.0;
c_r(4, :) =  [0.4, 5.0];

% model each obstacle with a seonf order ellipsoid
order_r = 2;

% Specify a specific size for obstacles
for i = 1:N_obs
    E_r(:,:,i) = diag(c_r(i,:));
    E1_r(:,:,i) = E_r(:,:,i)^(-1);
    E2_r(:,:,i) = E_r(:,:,i)^(-order_r);
end


% Number of agents to be controlled by our algorithm

for i = 1:N+N_obs
    if i <= N
        order(i) = order_a;
        rmin(i) = rmin_a;
        c(i,:) = c_a;
        E1(:,:,i) = E1_a;
        E2(:,:,i) = E2_a;
    else
        order(i) = order_r;
        rmin(i) = rmin_r(i - N);
        c(i,:) = c_r(i-N, :);
        E1(:,:,i) = E1_r(:,:, i - N);
        E2(:,:,i) = E2_r(:, :, i - N);
    end
end

% %%%%% for two agents
% po(:,:,1) = [1, 1];  po(:,:,2) = [-1, -1];
% pf(:,:,1) = [-1, -1];  pf(:,:,2) = [1, 1];
% % % for two obstacles for two robots
% po(:,:,3) = [0,0];
% po(:,:,4) = [0,0.5];
% po(:,:,4) = [0,-0.5];


% %%For 4 agents
po(:,:,1) = [1, 1];
po(:,:,2) = [1.5, 5];
po(:,:,3) = [5, 5];
po(:,:,4) = [5, 1];
pf(:,:,1) = [5, 5];
pf(:,:,2) = [5, 1];
pf(:,:,3) = [1,1];
pf(:,:,4) = [1, 5];

po(:,:,5) = [3,3];

%%%%%%%%%%%%%% CONSTRUCT SECOOND ORDER MODEL AND ASSOCIATED MATRICES %%%%%%%%%

model = get_model(h, model_params);
Lambda = get_lambda(model.A, model.B, k_hor, ndim);
A0 = get_a0(model.A, k_hor);

%%%%%%%%%%%%% CONSTRUCT MATRICES TO WORK WITH BEZIER CURVES %%%%%%%%%%%%%%%%%%%%%

% Beta - converts control points into polynomial coefficients
Beta = mat_bernstein2power(d, l, ndim);

% Gamma - sample the polynomial at different time steps
Gamma = mat_sample_poly(T_segment, 0:h:((k_hor-1)*h), d, l, ndim);

% Alpha - sum of squared derivatives of the position
cr = zeros(1, d+1); % weights on degree of derivative deg = [0 1 ... d]
cr(3) = cost_acc;
Alpha = mat_sum_sqrd_derivatives(d, T_segment, cr, l, ndim); % for energy minimization

%%%%%%%%%%%%%% CONSTRUCT TERMS OF THE COST FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The Hessian for the minimum snap cost function is
H_snap = Beta'*Alpha*Beta;

% For the goal tracking error cost function, define a weight matrix S
% Case 1: no collisions in the horizon - go fast         % Q matrix
S_free = s_free*[zeros(ndim*(k_hor-spd_f), ndim*k_hor);
    zeros(ndim*spd_f, ndim*(k_hor-spd_f)) eye(ndim*spd_f)];
% size(S_free)

% Case 2: collisions in the horizon - go slower
S_obs = s_obs*[zeros(ndim*(k_hor-spd_o), ndim*k_hor);
    zeros(ndim*spd_o, ndim*(k_hor-spd_o)) eye(ndim*spd_o)];
% size(S_obs)
% Case 3: colliding scenario, change setpoint and repel rapidly
S_repel = s_repel*[eye(ndim*spd_r) zeros(ndim*spd_r, ndim*(k_hor-spd_r));
    zeros(ndim*(k_hor-spd_r), ndim*k_hor)];
% size(S_repel)

Phi = Lambda.pos*Gamma*Beta;
Phi_vel = Lambda.vel*Gamma*Beta;
H_free = Phi'*S_free*Phi;
H_obs = Phi'*S_obs*Phi;
H_repel = Phi'*S_repel*Phi;

% The complete Hessian is simply the sum of the two
H_f = H_free + H_snap;
H_o = H_obs + H_snap;
H_r = H_repel + H_snap;

% The linear term of the cost function depends both on the goal location of
% agent i and on its current position and velocity
% We can construct the static part that depends on the desired location
for i = 1:N
    f_pf_free(:,:,i) = repmat((pf(:,:,i))', k_hor, 1)'*S_free*Phi;
    f_pf_obs(:,:,i) = repmat((pf(:,:,i))', k_hor, 1)'*S_obs*Phi;
end

% Predefine this matrix to construct f_pf_repel when needed
Rho_repel = S_repel*Phi;

% We can also construct the matrix that will then be multiplied by the
% initial condition -> X0'*A0'*S*Lambda*Gamma*Beta
mat_f_x0_free = A0.pos'*S_free*Lambda.pos*Gamma*Beta;
mat_f_x0_obs = A0.pos'*S_obs*Lambda.pos*Gamma*Beta;
mat_f_x0_repel = A0.pos'*S_repel*Lambda.pos*Gamma*Beta;

%%%%%%%%%%%%% CONSTRUCT INEQUALITY CONSTRAINT MATRICES %%%%%%%%%%%%%%%%%%%%%%

% Types of constraints: 1) Acceleration limits 2) workspace boundaries
% We need to create the matrices that map position control points into c-th
% derivative control points
T_ctrl_pts = cell_derivate_ctrl_pts(d);
[A_in, b_in] = build_ineq_constraints(d, l, h, ndim, k_hor, T_segment,...
    phys_limits, T_ctrl_pts, Beta);

%%%%%%%%%%%%% CONSTRUCT EQUALITY CONSTRAINT MATRICES %%%%%%%%%%%%%%%%%%%%

% Types of constraints: 1) Continuity constraints up to degree deg_poly
A_eq = build_eq_constraints(d, l, ndim, deg_poly, T_ctrl_pts);
% The constant vector b_eq will be updated within the solver function,
% since it requires knowledge of the initial condition of the reference

%%%%%%%%%%%%%% MATRICES TO DECODE SOLUTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% First construct all the matrices that map the solution vector to samples
% of the n-th derivative of position
model_s = get_model(Ts, model_params);
K_sample = length(0:Ts:h-Ts);
Lambda_s = get_lambda(model_s.A, model_s.B, K_sample, ndim);
A0_s = get_a0(model_s.A, K_sample);

for r = 0:d
    if r > 0
        Mu = T_ctrl_pts{r};
        Mu_nd = augment_array_ndim(Mu, ndim);
        Sigma_r = kron(eye(l), Mu_nd);
    else
        Sigma_r = eye(ndim*(d+1)*l);
    end
    
    Beta_r = mat_bernstein2power(d-r, l, ndim);
    
    % Sample Bezier curves at 1/h Hz for the whole horizon
    t_sample_r = 0:h:((k_hor-1)*h);
    Tau_r = mat_sample_poly(T_segment, t_sample_r, d-r, l, ndim);
    Der_h{r+1} = Tau_r*Beta_r*Sigma_r;
    
    % Sample Bezier curves at 1/Ts Hz for the first applied input
    t_sample_r = 0:Ts:h-Ts;
    Tau_r = mat_sample_poly(T_segment, t_sample_r, d-r, l, ndim);
    Der_ts{r+1} = Tau_r*Beta_r*Sigma_r;
end

% Sample states at 1/Ts Hz for the first applied input
t_sample_r = 0:Ts:h-Ts;
Tau_r = mat_sample_poly(T_segment, t_sample_r, d, l, ndim);
Phi_sample = Lambda_s.pos*Tau_r*Beta;
Phi_vel_sample = Lambda_s.vel*Tau_r*Beta;

%% %%%%%%%%%%%%%% INIT ALGORITHM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:N
    poi = po(:,:,i)';
    voi = 0.001*ones(ndim, 1);
    X0(:,i) = [poi; voi];
    pos_k_i(:,1,i) = poi;
    vel_k_i(:,1,i) = voi;
    pos_k_i_sample(:,1,i) = poi;
    X0_ref(:,:,i) = [poi, voi, zeros(ndim, d-1)];
    prev_state(:,i) = X0(:,i);
    for r = 1:deg_poly+1
        ref(:,1,r,i) = X0_ref(:,r,i);
        ref_sample(:,1,r,i) = X0_ref(:,r,i);
    end
    hor_ref(:,:,i,1) = repmat(poi, 1, k_hor);
    hor_rob(:,:,i,1) = repmat(poi, 1, k_hor+1);
    
    
    ref_k_i_h(:,1,i) = X0_ref(:,1,i);
    J_error(:,:,i) = 0;
    J_energy(:,:,i) = 0;
    J_voilation(:,:,i) = 0;
    J_total(:,:,i) = 0;
end

for i = N+1 : N + N_obs
    poi = po(:,:,i)';
    pos_k_i(:,1,i) = poi;
    hor_ref(:,:,i,1) = repmat(poi, 1, k_hor);
    hor_rob(:,:,i,1) = repmat(poi, 1, k_hor+1);
    X0_ref(:,:,i) = [poi, zeros(ndim,d)];
end



pred_X0 = X0;

% Variables for reference replanning based on state feedback
colors = distinguishable_colors(N);
tic
%% %%%%%%%%%%%%% MPC MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
continuous_rep_step = 0;


%% publisher and subscribers for four robots
tb3_pub = [rospublisher("/youbot_0/cmd_vel","geometry_msgs/Twist");
    rospublisher("/youbot_1/cmd_vel","geometry_msgs/Twist");
    rospublisher("/youbot_2/cmd_vel","geometry_msgs/Twist")
    rospublisher("/youbot_3/cmd_vel","geometry_msgs/Twist")];

%% ros subscribers for four robots
tb3_posesub = [rossubscriber('/youbot_0/states');
    rossubscriber('/youbot_1/states')
    rossubscriber('/youbot_2/states')
    rossubscriber('/youbot_3/states')];
pause(1)

main_loop_rate = rosrate(1/h);
for k = 2:K
    
    % Update states for the regular agents
    for i = 1:N
        % Compare the expected and sensed position at time k
        err_pos(:,k) = X0(1:2,i) - pred_X0(1:2,i);
        err_vel(:,k) = X0(3:4,i) - pred_X0(3:4,i);
        
        % Compare the current position and the reference
        err_pos_ref(:,k,i) = X0(1:2,i) - X0_ref(:,1,i);
        err_vel_ref(:,k,i) = X0(3:4,i) - X0_ref(:,2,i);
        
        der_err_ref(:,k,i) = (err_pos_ref(:,k,i) - err_pos_ref(:,k-1,i)) / h;
        
        % Cost that determines whether there's something disturbing the agent
        % Cost gets higher when the error between the reference and the state gets higher
        cost(:,k,i) = (err_pos_ref(:,k,i).^5) ./ -(X0(3:4,i)+sign(X0(3:4,i))*0.01);
        
        
        %         % Event Triggered Replanning
        trigger(k,i) = 0;
        if any(cost(:,k,i) > max_cost) || any(cost(:,k,i) < min_cost)
            %             max = any(cost(:,k,i) > max_cost)
            %             min = any(cost(:,k,i) < min_cost)
            %             cost(:,k,i)
            X0_ref(:,1,i) = X0(1:2,i);
            X0_ref(:,2,i) = X0(3:4,i);
            X0_ref(:,3:5,i) = zeros(2,3);
            trigger(k,i) = 1;
        else
            X0_ref(:,1,i) = X0_ref(:,1,i);
        end
        
        
        
        
        
        
        
        
        %% Include on-demand collision avoidance
        [A_coll, b_coll, pf_tmp, t_build(k,i)] = ondemand_softconstraints(hor_rob(:,2:end,:,k-1), Phi,...
            X0(:,i), A0.pos, i, rmin,...
            order, E1, E2, ndim);
        
        if ~isempty(b_coll) % collisions in the horizon
            % Include collision constraints and slack variables
            N_v = length(b_coll) / 3;
            A_in_i = [A_in zeros(size(A_in,1), N_v) ; A_coll];
            b_in_i = [b_in; b_coll];
            A_eq_i = [A_eq zeros(size(A_eq,1), N_v)];
            
            % Linear and quadratic term to penalize collision relaxation
            f_eps = lin_coll_penalty*ones(1, N_v);
            H_eps = quad_coll_penalty*eye(N_v);
            
            % If close to colliding, change setpoint to quickly react
            if ~isempty(pf_tmp) && false
                H_i = [H_r zeros(size(H_f,1), N_v)
                    zeros(N_v, size(H_f,2)) H_eps];
                mat_f_x0_i = mat_f_x0_repel;
                f_tot = repmat((pf_tmp),k_hor,1)'*Rho_repel;
            else
                H_i = [H_o zeros(size(H_f,1), N_v);
                    zeros(N_v,size(H_f,2)) H_eps];
                mat_f_x0_i = mat_f_x0_obs;
                f_tot = f_pf_obs(:,:,i);
            end
            
        else % no collisions in horizon
            A_in_i = A_in;
            b_in_i = b_in;
            A_eq_i = A_eq;
            H_i = H_f;
            f_eps = [];
            mat_f_x0_i = mat_f_x0_free;
            f_tot = f_pf_free(:,:,i);
        end
        
        
        %%
        
        % Solve QP
        t_start = tic;
        [sol, fval, exitflag] = softMPC_update(l, deg_poly, A_in_i, b_in_i, A_eq_i, H_i,...
            mat_f_x0_i, f_tot, f_eps, X0(:,i), X0_ref(:,:,i), ndim);
        
        t_qp(k,i) = toc(t_start);
        if  isempty(sol)
            fprintf("No solution found, using previous input \n")
            x = prev_x{i};
            %             assert(~isempty(x), 'ERROR: No solution found - exitflag =  %i\n',exitflag);
        else
            prev_x{i} = sol;
            x = sol;
        end
        
        % Extract the control points
        u = x(1:size(mat_f_x0_free, 2));
        
        % Apply input to model starting form our previous init condition
        % predicted  values
        pos_i = vec2mat(Phi*u + A0.pos*X0(:,i),ndim)';
        vel_i = vec2mat(Phi_vel*u + A0.vel*X0(:,i),ndim)';
        
        if ~isempty(b_coll) && debug_constr && use_ondemand
            figure(1)
            plot3(pos_i(1,:), pos_i(2,:), pos_i(3,:),...
                '*','Color',colors(i,:),'Linewidth',2);
            plot3(pf(1,1,i),pf(1,2,i),pf(1,3,i),'s','Color',colors(i,:),'Linewidth',4,'markers',10);
            fprintf("relaxed constraint by %.2f cm\n", epsilon*100);
        end
        
        % Sample at a higher frequency the interval 0:Ts:h-Ts
        % This tells us what should be the value of our state after
        % sending the optimal commands if the model was perfect
        % predicted values form X0 with time step Ts (0.01 sec)
        pos_i_sample = vec2mat(Phi_sample*u + A0_s.pos*X0(:,i), ndim)';
        vel_i_sample = vec2mat(Phi_vel_sample*u + A0_s.vel*X0(:,i),ndim)';
        
        
        
        %%%%%%%%% Publish data
        coppeliasim_robot_publisher(vel_i_sample, tb3_pub(i))
        
        
        % Sample the resulting reference Bezier curves at 1/h and 1/Ts
        % Get the next input to be applied 'X0_ref'
        cols = 2 + (k-2)*(h/Ts):1 + (k-1)*(h/Ts);
        for r = 1:d+1
            rth_ref(:,:,r) = vec2mat(Der_h{r}*u, ndim)';
            rth_ref_sample(:,:,r) = vec2mat(Der_ts{r}*u, ndim)';
            X0_ref(:,r,i) = rth_ref(:,2,r);
            ref(:,k,r,i) = rth_ref(:,2,r);
            ref_sample(:,cols,r,i) = rth_ref_sample(:,:,r);
        end
        
        
        
        
        
%%      Update feedback either from robot or from creating 
        %%%%%%%%%%% Create feedback with noise and disturbances
%         X0_ex(:,1) = X0(:,i);
%         for k_ex = 2:length(t_sample_r) + 1
%             X0_ex(:, k_ex -1) = X0_ex(:, k_ex -1)+ rnd_noise(std_p,std_v);
%             X0_ex(:,k_ex) = model_s.A*X0_ex(:, k_ex-1) + model_s.B*rth_ref_sample(:, k_ex-1, 1);
%         end
%         if ~disturbance || ~ismember(k,disturbance_k) || ~ismember(i,agent_disturb)
%             % Initial conditions for next MPC cycle - based on sensing
%             X0(:,i) = X0_ex(:, end);
%             
%             % Update agent's states at 1/h and 1/Ts frequencies
%             pos_k_i_sample(:,cols,i) = X0_ex(1:2, 2:end);
%             vel_k_i_sample(:,cols,i) = X0_ex(3:4, 2:end);
%             
%         elseif disturbance && ismember(k,disturbance_k) && ismember(i,agent_disturb)
%             X0(1,i) = X0(1,i);
%             X0(4,i) = 0;
%             X0(:,i) = X0(:,i) + rnd_noise(std_p,std_v);
%             
%             vel_k_i_sample(:,cols,i) = repmat(X0(3:4,i),1,h/Ts);
%         end
        %             pos_k_i_sample(:,cols,i) = repmat(X0(1:2,i),1,h/Ts);

        
        
        %%%%%%%%%%%%%  Take feed back from Robots in Coppliasim
        
        [X0(:,i)] = coppeliasim_robot_subscriber(tb3_posesub(i));
        % Update agent's states at 1/h and 1/Ts frequencies
        pos_k_i_sample(:,cols,i) = pos_i_sample;
        vel_k_i_sample(:,cols,i) = vel_i_sample;
        
        
%%
        
        
        
        
        
        pred_X0(:,i) = [pos_i_sample(:,end); vel_i_sample(:,end)];
        pos_k_i(:,k,i) = X0(1:2,i);
        vel_k_i(:,k,i) = X0(3:4,i);
        
        ref_k_i_h(:,k,i) = X0_ref(:,1,i);
        
        
        % Reference and state prediction horizons - visualization purposes
        hor_ref(:,:,i,k) = rth_ref(:,:,1);
        hor_rob_k(:,:,i) = [X0(1:2,i) pos_i(:,1:end)];
        
        
        
        
        %% Objective function
        % energy function
        J_energy(:,k,i) = u'*H_snap*u;
        
        J_error(:,k,i) = u'*H_free*u - 2*f_pf_free(:,:,i)*u + 2*X0(:,i)'*mat_f_x0_free*u;
        
        J_total(1,k,i) = fval;
        
        J_voilation(:,k,i) = fval - J_error(:,k,i) - J_energy(:,k,i);
        
    end
    
    
    
    
    
    
    
    % Update the states for the rogue agents
    for  i = N + 1:N
        % For now we're forcing the agents to remain in place
        pos_k_i(:,k,i) = pos_k_i(:,k-1,i);
        vel_k_i(:,k,i) = vel_k_i(:,k-1,i);
        hor_rob_k(:,:,i) = repmat(X0(1:3, i), 1, k_hor + 1);
    end
    
    % Update the states of the static obstacles
    for i = N+1 : N + N_obs
        poi = po(:,:,i)';
        pos_k_i(:,k,i) = poi;
        hor_rob_k(:,:, i) = repmat(poi, 1, k_hor + 1);
    end
    
    hor_rob(:,:,:,k) = hor_rob_k;
    
    
%     waitfor(main_loop_rate);
    
    
    
    
    
end
toc
% For Stoping Each Robots
vel_i_sample = zeros(ndim, K);
for i = 1:N
    coppeliasim_robot_publisher(vel_i_sample, tb3_pub(i))
end

%%% Ploting Results
%% For States and Positon

% figure(899)
% i=1;
% hold on
% plot(tk, pos_k_i(2,:,i), 'Linewidth',1.5, 'Color','b')
% plot(t, ref_sample(2,:,1,i), '--', 'Linewidth', 1.5, 'Color','r')
% plot(tk, cost(2,:,1),'g')
% ylabel("Y Position [m]")
% legend("States","Reference")
% title("Y Plot Without Replanning")
% hold off


% Position
% figure(900)
% subplot(3,1,1)
% hold on
% i=1; %robot i =1
% plot(tk, pos_k_i(1,:,i), 'Linewidth',1.5, 'Color','b')
% plot(tk, ref_k_i_h(1,:,1,i), '--', 'Linewidth', 1.5, 'Color','r')
% plot(tk, cost(1,:,1),'g')
% ylabel("X Position [m]")
% title("State vs Reference (Event Triggered)")
% legend("States","Reference")
% hold off
%
% subplot(3,1,2)
% hold on
% plot(tk, pos_k_i(2,:,i), 'Linewidth',1.5, 'Color','b')
% plot(tk, ref_k_i_h(2,:,1,i), '--', 'Linewidth', 1.5, 'Color','r')
% plot(tk, cost(2,:,1),'g')
% ylabel("Y Position [m]")
% legend("States","Reference")
% hold off
%
% subplot(3,1,3)
% hold on
% plot(tk, pos_k_i(3,:,i), 'Linewidth',1.5, 'Color','b')
% plot(tk, ref_k_i_h(3,:,1,i), '--', 'Linewidth', 1.5, 'Color','r')
% plot(tk, cost(3,:,1),'g')
% xlabel("Time [s]")
% ylabel("Z Position [m]")
% legend("States","Reference")
% hold off
%


%% Event Triggered Replanning
% %%%%%%%%%%%% Plot between error_ref and activation function
% figure(901)
% subplot(3,1,1)
% plot(tk, err_pos_ref(1,:,i), tk, cost(1,:,1,i),'g')
% legend("error", "activation function")
% ylabel("In X Position")
% title("Error and Activation Plots (Event Triggered)")
%
% subplot(3,1,2)
% plot(tk, err_pos_ref(2,:,i), tk, cost(2,:,1,i),'g')
% ylabel("In Y Position")
%
% subplot(3,1,3)
% plot(tk, err_pos_ref(3,:,i), tk, cost(3,:,1,i),'g')
% xlabel("Time [s]")
% ylabel("In Z position")


%
% %%%%%%%%%%%% Plot Rate of change of error_ref
% figure(9011)
% subplot(3,1,1)
% plot(tk, der_err_ref(1,:,i))
% xlabel("Time [s]")
% ylabel("In X position")
% title("Rate of change of Error Plot (Event Triggered)")
%
% subplot(3,1,2)
% plot(tk, der_err_ref(2,:,i))
% xlabel("Time [s]")
% ylabel("In Y position")
%
% subplot(3,1,3)
% plot(tk, der_err_ref(3,:,i))
% xlabel("Time [s]")
% ylabel("In Z position")
% title("Rate of change of error")

%
%%%%%%%%% Ploting variation in activation function with varing power from 1 to 10
% figure(9002)
% plot(tk, err_pos_ref(2,:), tk, cost1(2,:,1), tk, cost2(2,:,1), tk, cost3(2,:,1), tk, cost4(2,:,1),...
%      tk, cost5(2,:,1), tk, cost6(2,:,1), tk, cost7(2,:,1), tk, cost8(2,:,1), tk, cost9(2,:,1), tk, cost10(2,:,1))
% legend("error", "n=1", "n=2", "n=3", "n=4", "n=5", "n=6", "n=7", "n=8", "n=9", "n=10")
% xlabel("Time [s]")
% ylabel("activation function")
% title("variation in activation function with varing power from 1 to 10")
% % %
%
% %%%%%%%%% Ploting variation in activation function with varing power from 1
% %%%%%%%%% to 5
% figure(9003)
% plot(tk, err_pos_ref(2,:), tk, cost1(2,:,1), tk, cost2(2,:,1), tk, cost3(2,:,1), tk, cost4(2,:,1),...
%      tk, cost5(2,:,1))
% legend("error", "n=1", "n=2", "n=3", "n=4", "n=5")
% xlabel("Time [s]")
% ylabel("activation function")
% title("variation in activation function with varing power from 1 to 5")
% %
%
%
% %%%%%%%%% Ploting variation in activation function with varing power from 5 to 10
% figure(9004)
% plot(tk, err_pos_ref(2,:), tk, cost5(2,:,1), tk, cost6(2,:,1), tk, cost7(2,:,1), tk, cost8(2,:,1), tk, cost9(2,:,1), tk, cost10(2,:,1))
% legend("error", "n=5", "n=6", "n=7", "n=8", "n=9", "n=10")
% xlabel("Time [s]")
% ylabel("activation function")
% title("variation in activation function with varing power from 5 to 10")


%% For Objective Function
%%%%%% Ploting Objective functions
% figure(905)
% plot(tk, J_error(:,:,1),'r', tk, J_energy(:,:,1),'b-.',...
%         tk, J_voilation(:,:,1),'c', tk, J_total(:,:,1),'k',"LineWidth",1)
% legend("Error Cost", "Energy Cost","Voilation COst","Total Cost")
% xlabel("Time [s]")
% ylabel("Objective Cost functions")
% title("Objective Cost functions for Optimization (Event Triggered)")

rosshutdown
%% Ploting

ploting(N, view_states, view_cost, t, pos_k_i_sample, ref_sample, phys_limits, vel_k_i_sample, tk, cost, view_distance, ...
    E1, pos_k_i, order, rmin, visualize, K, N, hor_rob, pf, po, N_obs)