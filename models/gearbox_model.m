function [T_out, omega_out, J_reflected, P_loss] = gearbox_model(T_in, omega_in, params)
% GEARBOX_MODEL Model for multi-stage gearbox with losses
%
% Applies gear ratio transformation through multiple stages.
% Uses hybrid loss model: efficiency per stage + friction.
%
% Inputs:
%   T_in - Input torque [N·m]
%   omega_in - Input angular velocity [rad/s]
%   params - System parameters structure
%
% Outputs:
%   T_out - Output torque [N·m]
%   omega_out - Output angular velocity [rad/s]
%   J_reflected - Total reflected inertia [kg·m²]
%   P_loss - Total power loss [W]

% Extract parameters
ratios = params.gearbox.ratios;
eta = params.losses.gearbox_efficiency;
J_input = params.gearbox.inertia_input;
J_output = params.gearbox.inertia_output;
J_intermediate = params.gearbox.inertia_intermediate;
T_coulomb = params.losses.gearbox_coulomb;
B_viscous = params.losses.gearbox_viscous;

% Calculate total gearbox ratio
N_total = prod(ratios);

% Speed transformation
omega_out = omega_in / N_total;

% Power input
P_in = T_in * omega_in;

% Apply efficiency-based loss
% If multi-stage, efficiency compounds: eta_total = eta^n_stages
n_stages = length(ratios);
eta_total = eta^n_stages;
P_after_efficiency = P_in * eta_total;

% Calculate friction losses
T_friction = 0;
if abs(omega_in) > 1e-6
    T_friction = T_coulomb * sign(omega_in) + B_viscous * omega_in;
end

P_loss_friction = abs(T_friction * omega_in);
P_net = P_after_efficiency - P_loss_friction;
P_net = max(0, P_net);

% Output torque
if abs(omega_in) > 1e-6
    T_out = P_net / omega_out;
else
    T_out = (T_in * N_total * eta_total) - T_friction;
end

% Total power loss
P_loss = P_in - (T_out * omega_out);

% Reflected inertia calculation
% Reflect all inertias to input shaft
J_reflected = J_input;

% Reflect output inertia through total ratio
J_reflected = J_reflected + J_output / (N_total^2);

% Reflect intermediate inertias (if multi-stage)
if ~isempty(J_intermediate) && length(J_intermediate) == (n_stages - 1)
    N_cumulative = 1;
    for i = 1:length(J_intermediate)
        N_cumulative = N_cumulative * ratios(i);
        J_reflected = J_reflected + J_intermediate(i) / (N_cumulative^2);
    end
end

end
