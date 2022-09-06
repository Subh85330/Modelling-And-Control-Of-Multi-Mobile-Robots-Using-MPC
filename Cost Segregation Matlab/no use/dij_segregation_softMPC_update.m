function [x, fval, exitflag] = dij_segregation_softMPC_update(l, deg_poly, Ain, bin, Aeq, H,...
                                        mat_f_x0, f_tot, f_eps, X0, X0_ref, ndim, i, k_hor, f_seg_dij)
constr_tol = 1e-8;
options = optimoptions('quadprog', 'Display', 'off',...
                       'ConstraintTolerance', constr_tol);



%% Method 4 each group has a leader with fixed desired position

pf_1 = [1; -1]; pf_2 = [2;2];
mat_X0_pf = [X0(1:2,2), X0(1:2,1)];
f_tot_seg = repmat(mat_X0_pf(:,i), k_hor, 1)'*f_tot;

%%

% Construct linear term of the function based on X0 
% X0' = [px py pz vx vy vz]
f = [-2*(f_tot_seg + f_seg_dij(:,:,i) - X0(:,i)'*mat_f_x0), f_eps];

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
