function [A, b] = softBVC_constraints_ref(X0_ref, d, i, rmin, order, E1, E2, x_length)

% Construct the hyperspace constraints that will limit the first
% segment of the Bezier curve to lie within the BVC.
dh = d;
N = size(X0_ref, 3);
p_i = X0_ref(:, 1, i);
idx = 1;
A_coll = zeros(dh*(N-1), x_length);
b_coll = zeros(dh*(N-1), 1);
A = [];
b = [];
for j = 1:N
   if (i~= j)
       p_j = X0_ref(:, 1, j);
       dist = norm(E1(:,:,j)*(p_j-p_i),order(j));
       
       % Skip neighbour if it's far away
       if dist > 3 * rmin(j)
           continue
       end
       
       differ = (E2(:,:,j)*(p_i-p_j).^(order(j)-1))'; % Transpose of the difference
       
       % Right side of inequality constraint
       r = dist^(order(j)-1)*((rmin(j)) - dist + differ*p_i/(dist^(order(j)-1)));
       
       % the diff 1x3 vector must multiply all the control points of the
       % first segment of the Bezier curve, except the first one
       for  k = 1:dh
          A_coll((idx-1)*dh + k, 3*(k-1)+4:3*(k-1)+6) = -differ;
          b_coll((idx-1)*dh + k) = -r;
       end
       length_A = size(A, 1);
       A = [A zeros(length_A, dh); A_coll((idx-1)*dh + 1: idx*dh, :) zeros(dh, (idx-1)*dh) diag(ones(1, dh) * dist)];
       idx = idx + 1;
   end
end

nrows = (idx - 1) * dh;
A = [A; zeros(nrows, x_length) eye(nrows); zeros(nrows, x_length) -eye(nrows)];
b = [b_coll(1:nrows); zeros(nrows, 1); 20*ones(nrows, 1)];