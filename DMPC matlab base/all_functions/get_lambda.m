function Lambda = get_lambda(A, B, K, ndim)
Lambda.pos = [];
Lambda.vel = [];
prev_row = zeros(4, ndim*K); 
ncols = size(B, 2);
nrows = size(B, 1);
for k = 1:K
    add_B = [zeros(nrows, ncols*(k-1)), B, zeros(nrows, ncols*(K-k))];
    new_row = A*prev_row + add_B;   
    Lambda.pos = [Lambda.pos; new_row(1:2,:)];
    Lambda.vel = [Lambda.vel; new_row(3:4,:)];
    prev_row = new_row;   
end