function [T_motor, V_emf, V_terminal, P_electrical, P_losses] = motor_model(omega_motor, I, params)
% MOTOR_MODEL Model for SK3-5065-236KV motor operating in generator mode
%
% Calculates back-EMF, terminal voltage, torque, and losses.
% Motor operates as generator: mechanical power in, electrical power out.
%
% Inputs:
%   omega_motor - Motor angular velocity [rad/s]
%   I - Motor current [A]
%   params - System parameters structure
%
% Outputs:
%   T_motor - Motor torque (opposing motion in generator mode) [N·m]
%   V_emf - Back-EMF voltage [V]
%   V_terminal - Terminal voltage [V]
%   P_electrical - Electrical power output [W]
%   P_losses - Total motor losses [W]

% Extract motor parameters
K_e = params.motor.K_e;
K_t = params.motor.K_t;
R_winding = params.motor.R_winding;
core_loss_coeff = params.motor.core_loss_coefficient;
mech_loss_coeff = params.motor.mechanical_loss_coefficient;
T_coulomb = params.losses.motor_bearing_coulomb;
B_viscous = params.losses.motor_bearing_viscous;

% Back-EMF (proportional to speed)
V_emf = K_e * omega_motor;

% Terminal voltage (generator convention: V_terminal = V_emf - I*R)
V_terminal = V_emf - I * R_winding;

% Motor torque (opposing motion in generator mode)
% Positive current produces negative torque (braking)
T_motor = -K_t * I;  % Negative sign: generator opposes motion

% Electrical power output (positive when generating)
P_electrical = V_terminal * I;

% Loss calculations
% Copper losses (I²R)
P_cu = I^2 * R_winding;

% Core losses (iron losses, eddy currents) - speed dependent
P_core = core_loss_coeff * omega_motor^2;

% Mechanical losses (bearing friction, windage)
T_mech_loss = 0;
if abs(omega_motor) > 1e-6
    T_mech_loss = T_coulomb * sign(omega_motor) + B_viscous * omega_motor;
end
P_mech_loss = abs(T_mech_loss * omega_motor);

% Total losses
P_losses = P_cu + P_core + P_mech_loss;

% Adjust motor torque to account for mechanical losses
T_motor = T_motor - T_mech_loss * sign(omega_motor);

end
