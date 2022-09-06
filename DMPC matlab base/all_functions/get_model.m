function model = get_model(h, model_params)

% This is bad practice, but here we're doing it a safe way for a specific purpose
% Unpack the structure model_params

% names = fieldnames(model_params);
% for i = 1:length(names)
%    eval([names{i} '=model_params.' names{i} ';']); 
% end


% Matrices of the discrete state space model

% for 2 dimensional case
% State = [x y vx vy vz]
model.A = [1  0 h 0;
           0 1 0 h;
           -h*model_params.omega_xy^2 0 1-2*model_params.omega_xy*h*model_params.zeta_xy 0;
           0 -h*model_params.omega_xy^2 0 1-2*model_params.omega_xy*h*model_params.zeta_xy];
 
model.B = [zeros(2, 2);
           h*model_params.omega_xy^2 0;
           0 h*model_params.omega_xy^2];