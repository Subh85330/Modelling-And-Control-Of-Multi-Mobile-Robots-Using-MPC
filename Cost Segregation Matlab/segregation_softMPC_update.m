function [x, fval, exitflag] = segregation_softMPC_update(l, deg_poly, Ain, bin, Aeq, H,...
                                        mat_f_x0, f_tot, f_eps, X0, X0_ref, ndim, i, k_hor, mat_X0_pf)
constr_tol = 1e-8;
options = optimoptions('quadprog', 'Display', 'off',...
                       'ConstraintTolerance', constr_tol);

%% Method 0 each agent has a leader (1->2, 2->3, 3->1) works only for 2 robots
% n = size(X0, 2);
% f_tot_seg = zeros(1, size(f_tot,2));
% f_tot_seg = [];
% for j=1:n
%     if i~=j
%         f_tot_seg =  repmat(X0(1:2,j), 16, 1)'*f_tot;
%     end
% end

%% Method 2 each agent has a leader (1->2, 2->3, 3->1)
% mat_X0_pf = [X0(1:2,2), X0(1:2,3), X0(1:2,1)];
% f_tot_seg = repmat(mat_X0_pf(:,i), k_hor, 1)'*f_tot;


%% Method 3 each agent is following average of other robots
fixt_pos2 = [-3;3]; fixt_pos1 = [6;6];
X0(1:2,1) = X0(1:2,1)+fixt_pos1;
X0(1:2,4) = X0(1:2,4)+fixt_pos2;

% mat_X0_pf = [2*fixt_pos1, X0(1:2,3)+X0(1:2,1), X0(1:2,1)+X0(1:2,2), 2*fixt_pos2, X0(1:2,6)+X0(1:2,4), X0(1:2,5)+X0(1:2,4) ]./2;
% mat_X0_pf = [X0(1:2,2)+X0(1:2,3)+X0(1:2,1), X0(1:2,2)+X0(1:2,3)+X0(1:2,1), X0(1:2,2)+X0(1:2,3)+X0(1:2,1),...
%              X0(1:2,4)+X0(1:2,5)+X0(1:2,6), X0(1:2,4)+X0(1:2,5)+X0(1:2,6), X0(1:2,4)+X0(1:2,5)+X0(1:2,6)]./4;
% mat_X0_pf = [fixt_pos, fixt_pos, fixt_pos];
f_tot_seg = repmat(mat_X0_pf(:,i), k_hor, 1)'*f_tot;


%% Method 4 each group has a leader with fixed desired position

% pf_1 = [1; -1]; pf_2 = [2;2];
% mat_X0_pf = [pf_1, X0(1:2,3), X0(1:2,1), pf_2, X0(1:2,6), X0(1:2,4)];
% f_tot_seg = repmat(mat_X0_pf(:,i), k_hor, 1)'*f_tot;

%%

% Construct linear term of the function based on X0 
% X0' = [px py pz vx vy vz]
f = [-2*(f_tot_seg - X0(:,i)'*mat_f_x0), f_eps]; 

% Construct the vector beq based on X0_ref
% X0_ref = [px vx ax jx sx]
%          |py vy ay jy sy|
%          [pz vz az jz sz]
beq = zeros(size(Aeq, 1), 1);
for i = 1:deg_poly + 1
    beq(ndim*(i-1)*l + 1 : ndim*((i-1)*l + 1)) = X0_ref(:,i);
end

% Solve the QP
[x, fval, exitflag] = quadprog(2*H, f', Ain, bin, Aeq, beq, [], [], [], options);
