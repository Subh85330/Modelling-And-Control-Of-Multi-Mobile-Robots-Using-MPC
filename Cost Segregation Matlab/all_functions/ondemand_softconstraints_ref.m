function [A_coll, b_coll, pf_tmp, t_elapsed] = ondemand_softconstraints_ref(hor_ref, Phi_ref, X0, i,...
                                                                            rmin, order, E1, E2)
                                                         
t_start = tic;
A_coll = [];
b_coll = [];
K = size(hor_ref, 2);
relax_lim = 20000;
pf_tmp = [];
% Detect first collision on the horizon and return constraint tuple
for k = 1:K
    hor_k = squeeze(hor_ref(:,k,:));
    [collisions, neighbrs] = check_horizon(hor_k, i, E1, rmin, order); 
    
    if (any(collisions))
        k_ctr = k;
        N_v = sum(neighbrs);
        [A_coll, b_coll, dist, pf_tmp] = build_constraint_ref(hor_k, k, neighbrs, X0, Phi_ref,...
                                                          i, rmin, order, E1, E2);
        ncols = size(A_coll, 2);
%         fprintf("k_ctr = %i with dist = %.2f m \n", k_ctr, min(dist));
        A_coll = [A_coll diag(dist);
                  zeros(N_v, ncols) eye(N_v);
                  zeros(N_v, ncols) -eye(N_v)];

        b_coll = [b_coll; zeros(N_v, 1); relax_lim*ones(N_v, 1)];
        break;
    end  
end

t_elapsed = toc(t_start);
% fprintf("check horizon time = %.2f ms\n", t_elapsed*1000)

colors = distinguishable_colors(2);
global debug_constr;

if (any(collisions)) && debug_constr
    viol_idx = find(collisions == 1);
    num_coll = length(viol_idx);
    figure(1)
    [az,el] = view;
    clf
    for n=1:num_coll
       p1 = hor_k(:,i);
       p2 = hor_k(:,viol_idx(n));
       diff = (p2 - p1);
       w = null(diff');
       new_p = p2 - E1^(-1)*diff/norm(diff)*rmin;
       [P,Q] = meshgrid(-1:1); % Provide a gridwork (you choose the size)
       X = new_p(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
       Y = new_p(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
       Z = new_p(3)+w(3,1)*P+w(3,2)*Q;
       surf(X,Y,Z)
       hold on;
       xlim([-3,3])
       ylim([-3,3])
       zlim([-3,3])

       plot3(hor_ref(1,:,i),hor_ref(2,:,i),hor_ref(3,:,i),...
           'o','Color',colors(i,:),'Linewidth',1)
       plot3(p1(1),p1(2),p1(3),'o','Color',colors(i,:),'Linewidth',4);
       plot3(p2(1),p2(2),p2(3),'o','Color',colors(viol_idx(n),:),'Linewidth',4);
       plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)])
    end
    
end