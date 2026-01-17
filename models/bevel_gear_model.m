function [T_out, omega_out, J_reflected, P_loss] = bevel_gear_model(T_in, omega_in, params)
% BEVEL_GEAR_MODEL Model for bevel gear system (1:1 ratio, axis redirection)
%
% Bevel gears provide 1:1 ratio but introduce losses and inertia.
% No speed change, only losses and inertia effects.
%
% Inputs:
%   T_in - Input torque [N·m]
%   omega_in - Input angular velocity [rad/s]
%   params - System parameters structure
%
% Outputs:
%   T_out - Output torque [N·m]
%   omega_out - Output angular velocity [rad/s]
%   J_reflected - Reflected inertia [kg·m²]
%   P_loss - Power loss [W]

% Extract parameters
N = params.bevel.ratio;  % Should be 1.0
eta = params.losses.bevel_efficiency;
J_bevel = params.bevel.inertia;
T_coulomb = params.losses.bevel_coulomb;
B_viscous = params.losses.bevel_viscous;

% Speed transformation (1:1 ratio)
omega_out = omega_in / N;

% Power input
P_in = T_in * omega_in;

% Apply efficiency-based loss
P_after_efficiency = P_in * eta;

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
    T_out = (T_in * N * eta) - T_friction;
end

% Total power loss
P_loss = P_in - (T_out * omega_out);

% Reflected inertia (1:1 ratio, so no transformation needed)
J_reflected = J_bevel;

end
