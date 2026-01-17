function [T_grav, J_eff, h, v_linear] = mass_drum_model(theta, omega, params)
% MASS_DRUM_MODEL Model for falling mass and drum system
%
% Converts linear motion of falling mass to rotational motion of drum.
% Applies gravitational torque and accounts for drum inertia.
%
% Inputs:
%   theta - Angular position of drum [rad]
%   omega - Angular velocity of drum [rad/s]
%   params - System parameters structure
%
% Outputs:
%   T_grav - Gravitational torque on drum [N·m]
%   J_eff - Effective moment of inertia (mass + drum) [kg·m²]
%   h - Current height of mass [m]
%   v_linear - Linear velocity of mass [m/s]

% Extract parameters
m = params.physical.mass;
g = params.physical.g;
r_drum = params.physical.drum_radius;
J_drum = params.physical.drum_inertia;
h_initial = params.physical.initial_height;

% Calculate linear position and velocity
h = h_initial - r_drum * theta;  % Height decreases as theta increases
v_linear = r_drum * omega;

% Ensure height doesn't go negative
h = max(0, h);

% Gravitational torque (force * radius)
% When mass is falling, it applies torque in positive theta direction
T_grav = m * g * r_drum;

% Effective moment of inertia
% Mass contributes: J_mass = m * r_drum^2 (reflected to drum)
J_mass = m * r_drum^2;
J_eff = J_drum + J_mass;

end
