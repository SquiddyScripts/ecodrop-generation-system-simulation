function [T_out, omega_out, J_reflected, P_loss] = sprocket_model(T_in, omega_in, params)
% SPROCKET_MODEL Model for sprocket stage with losses
%
% Applies gear ratio transformation and losses (efficiency + friction).
% Uses hybrid loss model: efficiency for gear mesh, friction for bearings.
%
% Inputs:
%   T_in - Input torque [N·m]
%   omega_in - Input angular velocity [rad/s]
%   params - System parameters structure
%
% Outputs:
%   T_out - Output torque [N·m]
%   omega_out - Output angular velocity [rad/s]
%   J_reflected - Reflected inertia from sprocket [kg·m²]
%   P_loss - Power loss [W]

% Extract parameters
N = params.physical.sprocket_ratio;
eta = params.losses.sprocket_efficiency;
J_sprocket = params.physical.sprocket_inertia;
T_coulomb = params.losses.sprocket_coulomb;
B_viscous = params.losses.sprocket_viscous;

% Speed transformation
omega_out = omega_in / N;  % Output speed = input speed / ratio

% Calculate friction losses
T_friction = 0;
if abs(omega_in) > 1e-6  % Only apply friction if moving
    T_friction = T_coulomb * sign(omega_in) + B_viscous * omega_in;
end

% Power input
P_in = T_in * omega_in;

% Apply efficiency-based loss (gear mesh)
P_after_efficiency = P_in * eta;

% Apply friction losses
P_loss_friction = abs(T_friction * omega_in);
P_net = P_after_efficiency - P_loss_friction;
P_net = max(0, P_net);  % Can't have negative power

% Output torque (accounting for ratio and losses)
if abs(omega_in) > 1e-6
    T_out = P_net / omega_out;
else
    % Static case: torque limited by friction
    T_out = (T_in * N * eta) - T_friction;
end

% Total power loss
P_loss = P_in - (T_out * omega_out);

% Reflected inertia (sprocket inertia seen at input)
J_reflected = J_sprocket;

end
