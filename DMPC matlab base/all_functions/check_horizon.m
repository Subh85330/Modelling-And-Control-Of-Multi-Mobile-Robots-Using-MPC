function [collisions, neighbours] = check_horizon(hor_k, i, E1, rmin, order, ndim)
N = size(hor_k, 2);
collisions = zeros(N, 1);
neighbours = zeros(N, 1);

for j = 1:N %Iterate through the number of obstacles (other agents)
    if(i~=j)
        dist = norm(E1(1:ndim,1:ndim,j)*(hor_k(:,i) - hor_k(:,j)), order(j));
        collisions(j) = (dist < rmin(j));
        neighbours(j) = dist < rmin(j)*2;
    end  
end
