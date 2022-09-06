function model = get_model(h, model_params)

% State = [x y vx vy vz]
model.A = [1  0 h 0;
           0 1 0 h;
           -h*model_params.omega_xy^2 0 1-2*model_params.omega_xy*h*model_params.zeta_xy 0;
           0 -h*model_params.omega_xy^2 0 1-2*model_params.omega_xy*h*model_params.zeta_xy];
 
model.B = [zeros(2, 2);
           h*model_params.omega_xy^2 0;
           0 h*model_params.omega_xy^2];