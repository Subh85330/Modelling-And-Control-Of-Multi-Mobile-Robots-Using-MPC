function [Ain_k, bin_k, prev_dist, pf_tmp] = build_constraint_ref(hor_k, k, neighbours, X0, Phi,...
                                                                  i, rmin, order, E1, E2)

N = size(hor_k, 2);
K = size(Phi,1) / 3;
N_coll = sum(neighbours);
Ain_k = zeros(N_coll, size(Phi,2));
bin_k = zeros(N_coll, 1);
k_ctr = k;
p_i = hor_k(:,i);
idx = 1;
pf_tmp = [];
if k_ctr == 0
    k_ctr = 1;
end

if k_ctr >= K
    k_ctr = K - 1;
end
% k_ctr

% Loop through all the colliding neighbours and append the contraint tuple (A, b)
for j = 1:N
   if (i~= j && neighbours(j))
       p_j = hor_k(:,j);
       dist = norm(E1(:,:,j)*(p_i-p_j), order(j));
       differ = (E2(:,:,j)*(p_i-p_j).^(order(j)-1))'; 
       prev_dist(idx) = dist^(order(j)-1);
       
       if k_ctr <=1 && dist < rmin(j) - 0.05
           % We're almost crashing, force agents to repel each other
           pf_tmp = p_i + (p_i-p_j)*(rmin(j)+0.1 -dist)/dist;
       end
       
       % Build intermediate terms for matrices
       zeta = dist^(order(j)-1)*(rmin(j) - dist);
       rho = zeta + differ*p_i;
       diff_mat = [zeros(1, 3*(k_ctr-1)) differ zeros(1, 3*(K-k_ctr))];
       
       % Build inequality constraint Ain_k*x <= bin_k
       Ain_k(idx,:) = -diff_mat*Phi;
       bin_k(idx) = -rho;
       idx = idx + 1;
   end
end
