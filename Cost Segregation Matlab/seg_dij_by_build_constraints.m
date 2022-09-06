function [A_dij_in, b_dij_in] = seg_dij_by_build_constraints(i, X0, hor_rob, order, rmin, A0, ndim, Phi, E1, E2)



k_ctr =3;
hor_k = squeeze(hor_rob(:,k_ctr,:));
N = size(hor_k, 2);
K = size(A0,1) / ndim;
p_i = hor_k(:,i);
% A_dij_in = zeros(N, size(Phi,2));
% b_dij_in = zeros(N, 1);
idx = 1;
for j = 1:N
   if (i~= j)
       p_j = X0(1:2,j);
       dist = norm(E1(:,:,j)*(p_i-p_j), order(j));
       differ = (E2(:,:,j)*(p_i-p_j).^(order(j)-1))';
       prev_dist(idx) = dist^(order(j)-1);
       
%        if k_ctr <=1 && dist < rmin(j) - 0.05
%            % We're almost crashing, force agents to repel each other
%            pf_tmp = p_i + (p_i-p_j)*(rmin(j)+0.1 -dist)/dist
%        end
       
       % Build intermediate terms for matrices
       init_cond = differ * A0(ndim *(k_ctr-1) + 1 : ndim*k_ctr, :)*X0(:,i);
       zeta = dist^(order(j)-1)*(1 - dist);
       rho = zeta + differ*p_i - init_cond;
       diff_mat = [zeros(1, ndim*(k_ctr-1)) differ zeros(1, ndim*(K-k_ctr))];
       
       % Build inequality constraint Ain_k*x <= bin_k
       A_dij_in(idx,:) = -diff_mat*Phi;
       b_dij_in(idx,:) = -rho;
       idx = idx + 1;
   end
   
end