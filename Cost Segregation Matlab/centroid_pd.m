function mat_X0_pf = centroid_pd(po_goal_groups, no_of_agents_in_groups, X0 )
No_of_groups = length(po_goal_groups);
N = sum(no_of_agents_in_groups);   % Total agents



mat_X0_pf = [];
ind_nxt_gp = 0;
for gi = 1:No_of_groups
    centroid = [];
    centroid = po_goal_groups(gi,:);
    for i = 1:no_of_agents_in_groups(gi)
%         po(:,:,i);
        centroid = centroid + X0(:,i+ind_nxt_gp)';
    end
    ci_gp = centroid./(no_of_agents_in_groups(gi)+1);
    for i = 1:no_of_agents_in_groups(gi)
        mat_X0_pf = [mat_X0_pf, ci_gp'];
        ind_nxt_gp = ind_nxt_gp+1;
    end
    
    
end
    
    
    
    
    
        %{
    po_goal_groups - is the position of goal point for each groups
    no_of_agents_in_groups - is the numbers of agents in each group
    %}
end