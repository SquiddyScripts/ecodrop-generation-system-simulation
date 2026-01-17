function J_reflected = reflect_inertia(J_actual, N_ratio)
% REFLECT_INERTIA Reflect moment of inertia through a gear ratio
%
% When an inertia is connected through a gear ratio, its effective
% contribution at the input shaft is J_actual / N^2, where N is the
% gear ratio (output/input speed ratio).
%
% Inputs:
%   J_actual - Actual moment of inertia [kg·m²]
%   N_ratio - Gear ratio (output speed / input speed)
%
% Outputs:
%   J_reflected - Reflected inertia at input shaft [kg·m²]

if N_ratio == 0
    error('Gear ratio cannot be zero');
end

J_reflected = J_actual / (N_ratio^2);

end
