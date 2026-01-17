function [dstate, component_outputs] = system_ode(t, state, params)
% SYSTEM_ODE ODE function for the electromechanical system
%
% Assembles all components, calculates torques, applies gear ratios/losses,
% and returns state derivatives.
%
% State vector:
%   state(1) = theta_mass: Angular position of drum [rad]
%   state(2) = omega_mass: Angular velocity of drum [rad/s]
%   state(3) = omega_motor: Angular velocity of motor [rad/s]
%   state(4) = I_motor: Motor current [A] (if electrical dynamics included)
%
% Inputs:
%   t - Current time [s]
%   state - State vector
%   params - System parameters structure
%
% Outputs:
%   dstate - State derivatives

% Extract state variables
theta_mass = state(1);
omega_mass = state(2);
omega_motor = state(3);

% Check if electrical dynamics are included
include_electrical_dynamics = params.simulation.include_electrical_dynamics;

if include_electrical_dynamics && length(state) >= 4
    I_motor = state(4);
else
    % Quasi-static: calculate current from load model
    % First need motor speed to calculate back-EMF
    % This creates a dependency - we'll solve iteratively or use previous value
    % For now, use omega_motor to estimate
    V_emf_est = params.motor.K_e * omega_motor;
    I_motor = load_model(V_emf_est, t, params);
end

%% 1. Mass & Drum Model
[T_grav, J_drum_eff, h, v_linear] = mass_drum_model(theta_mass, omega_mass, params);

% Check if mass has reached bottom
if h <= 0 && params.simulation.stop_at_bottom
    dstate = zeros(size(state));
    % Assign component_outputs if requested (for consistency with ODE solver calls)
    if nargout > 1
        component_outputs = struct();
        component_outputs.h = 0;
        component_outputs.v_linear = 0;
        component_outputs.T_grav = 0;
        component_outputs.J_drum_eff = J_drum_eff;
        component_outputs.P_loss_sprocket = 0;
        component_outputs.P_loss_gearbox = 0;
        component_outputs.P_loss_bevel = 0;
        component_outputs.P_electrical = 0;
        component_outputs.P_losses_motor = 0;
        component_outputs.I_motor = 0;
        component_outputs.V_terminal = 0;
        component_outputs.omega_motor = omega_motor;  % Use current state value
    end
    return;
end

%% 2. Sprocket Model
[T_sprocket_out, omega_sprocket, J_sprocket_reflected, P_loss_sprocket] = ...
    sprocket_model(T_grav, omega_mass, params);

%% 3. Gearbox Model
[T_gearbox_out, omega_gearbox, J_gearbox_reflected, P_loss_gearbox] = ...
    gearbox_model(T_sprocket_out, omega_sprocket, params);

%% 4. Bevel Gear Model
[T_bevel_out, omega_bevel, J_bevel_reflected, P_loss_bevel] = ...
    bevel_gear_model(T_gearbox_out, omega_gearbox, params);

% Bevel output should match motor input (1:1 ratio)
omega_motor_calc = omega_bevel;

%% 5. Motor Model
[T_motor, V_emf, V_terminal, P_electrical, P_losses_motor] = ...
    motor_model(omega_motor_calc, I_motor, params);

%% 6. Electrical Load Model (if not using electrical dynamics)
if ~include_electrical_dynamics
    I_motor = load_model(V_terminal, t, params);
    % Recalculate motor torque with updated current
    [T_motor, ~, ~, ~, ~] = motor_model(omega_motor_calc, I_motor, params);
end

%% 7. Calculate Total Inertias (reflected to each shaft)

% Total inertia at drum (mass + drum + reflected sprocket)
J_total_drum = J_drum_eff + reflect_inertia(J_sprocket_reflected, params.physical.sprocket_ratio);

% Total inertia at sprocket output (reflected gearbox)
N_sprocket = params.physical.sprocket_ratio;
J_total_sprocket_out = J_gearbox_reflected / (N_sprocket^2);

% Total inertia at gearbox output (reflected bevel)
N_gearbox_total = prod(params.gearbox.ratios);
J_total_gearbox_out = J_bevel_reflected / (N_gearbox_total^2);

% Total inertia at motor (bevel + motor rotor)
J_total_motor = J_bevel_reflected + params.motor.J_rotor;

%% 8. Torque Balance and Accelerations

% The system is kinematically constrained: all speeds are related through gear ratios
% Use omega_mass as the primary state, calculate all other speeds from it
N_sprocket = params.physical.sprocket_ratio;
N_gearbox_total = prod(params.gearbox.ratios);
N_bevel = params.bevel.ratio;
N_total = N_sprocket * N_gearbox_total * N_bevel;

% Enforce kinematic constraint: omega_motor = omega_mass / N_total
omega_motor = omega_mass / N_total;

% Recalculate motor model with correct speed
[T_motor, V_emf, V_terminal, P_electrical, P_losses_motor] = ...
    motor_model(omega_motor, I_motor, params);

% Recalculate load current with correct terminal voltage
if ~include_electrical_dynamics
    I_motor = load_model(V_terminal, t, params);
    [T_motor, ~, ~, ~, ~] = motor_model(omega_motor, I_motor, params);
end

% Calculate total effective inertia at drum shaft
% Reflect all inertias through gear ratios to drum
J_motor_reflected = params.motor.J_rotor / (N_total^2);
J_bevel_reflected_at_drum = J_bevel_reflected / (N_total^2);
J_gearbox_reflected_at_drum = J_gearbox_reflected / ((N_sprocket * N_gearbox_total)^2);
J_sprocket_reflected_at_drum = J_sprocket_reflected / (N_sprocket^2);

J_total_effective = J_drum_eff + J_sprocket_reflected_at_drum + ...
                    J_gearbox_reflected_at_drum + J_bevel_reflected_at_drum + ...
                    J_motor_reflected;

% Reflect motor torque back to drum shaft
% Motor torque opposes motion, so it acts as a load
T_motor_reflected = abs(T_motor) * N_total;  % Reflected to drum (opposing)

% Net torque at drum
T_net_drum = T_grav - T_motor_reflected;

% Angular acceleration at drum
alpha_mass = T_net_drum / J_total_effective;

% Motor acceleration (from kinematic constraint)
alpha_motor = alpha_mass / N_total;

%% 9. Electrical Dynamics (if included)
if include_electrical_dynamics
    % dI/dt = (V_emf - V_terminal - I*R) / L
    % But V_terminal is determined by load model
    % This creates an algebraic loop - need to solve iteratively
    % Simplified: assume load responds instantaneously
    L = params.motor.L_winding;
    if L > 0
        dI_dt = (V_emf - V_terminal - I_motor * params.motor.R_winding) / L;
    else
        dI_dt = 0;  % Quasi-static
    end
else
    dI_dt = 0;  % Not a state variable
end

%% 10. State Derivatives
dstate = zeros(size(state));
dstate(1) = omega_mass;           % dtheta/dt = omega
dstate(2) = alpha_mass;           % domega_mass/dt = alpha_mass
dstate(3) = alpha_motor;          % domega_motor/dt = alpha_motor

if include_electrical_dynamics && length(state) >= 4
    dstate(4) = dI_dt;             % dI/dt
end

% Apply current limit
if I_motor > params.motor.I_max
    % Reduce acceleration to limit current
    % This is a simplified approach - in reality, would need current control
    dstate(3) = dstate(3) * (params.motor.I_max / I_motor);
end

%% Store component outputs for energy analysis
if nargout > 1
    component_outputs = struct();
    component_outputs.h = h;
    component_outputs.v_linear = v_linear;
    component_outputs.T_grav = T_grav;
    component_outputs.J_drum_eff = J_drum_eff;
    component_outputs.P_loss_sprocket = P_loss_sprocket;
    component_outputs.P_loss_gearbox = P_loss_gearbox;
    component_outputs.P_loss_bevel = P_loss_bevel;
    component_outputs.P_electrical = P_electrical;
    component_outputs.P_losses_motor = P_losses_motor;
    component_outputs.I_motor = I_motor;
    component_outputs.V_terminal = V_terminal;
    component_outputs.omega_motor = omega_motor;
else
    component_outputs = [];
end

end
