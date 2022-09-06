function T_sample_poly = get_sample_mat(Tau, num_samples, dim, ndim)
l = length(Tau);
N = dim + 1;
T_sample_poly = zeros(ndim*num_samples, ndim*N*l); %ncols = nD * N ctrl pts * l
Tau_nd{l} = [];
curr_row = 1;
for i = 1:l
    if ~isempty(Tau{i})
        Tau_nd{i} = augment_array_ndim(Tau{i}, ndim);
        nrow = size(Tau_nd{i}, 1);
        ncol = size(Tau_nd{i}, 2);
        rows = curr_row : curr_row + nrow - 1;
        cols = (i-1)*ncol + 1 : i*ncol;
        T_sample_poly(rows, cols) = Tau_nd{i};
        curr_row = curr_row + nrow;
    end
end