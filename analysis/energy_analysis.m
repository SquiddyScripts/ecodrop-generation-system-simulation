function energy = energy_analysis(t, state, params, component_outputs)
% ENERGY_ANALYSIS Calculate energy flow and losses at current time step
%
% Tracks gravitational, kinetic, electrical energy and losses.
% Called at each ODE evaluation to build energy history.
%
% Inputs:
%   t - Current time [s]
%   state - Current state vector
%   params - System parameters structure
%   component_outputs - Structure with component outputs (torques, powers, etc.)
%
% Outputs:
%   energy - Structure containing all energy metrics

% Extract state
theta_mass = state(1);
omega_mass = state(2);
omega_motor = state(3);

% Extract component outputs
h = component_outputs.h;
v_linear = component_outputs.v_linear;
T_grav = component_outputs.T_grav;
J_drum_eff = component_outputs.J_drum_eff;
P_loss_sprocket = component_outputs.P_loss_sprocket;
P_loss_gearbox = component_outputs.P_loss_gearbox;
P_loss_bevel = component_outputs.P_loss_bevel;
P_electrical = component_outputs.P_electrical;
P_losses_motor = component_outputs.P_losses_motor;
I_motor = component_outputs.I_motor;
V_terminal = component_outputs.V_terminal;

% Physical parameters
m = params.physical.mass;
g = params.physical.g;
h_initial = params.physical.initial_height;
r_drum = params.physical.drum_radius;

% Gear ratios
N_sprocket = params.physical.sprocket_ratio;
N_gearbox_total = prod(params.gearbox.ratios);
N_bevel = params.bevel.ratio;
N_total = N_sprocket * N_gearbox_total * N_bevel;

%% Gravitational Potential Energy
E_gravitational = m * g * h;

%% Rotational Kinetic Energy
% Kinetic energy at drum
E_kin_drum = 0.5 * J_drum_eff * omega_mass^2;

% Kinetic energy at motor (reflected)
E_kin_motor = 0.5 * params.motor.J_rotor * omega_motor^2;

% Total kinetic energy (all components)
% For simplicity, sum major contributions
E_kinetic = E_kin_drum + E_kin_motor;

%% Electrical Energy
% Electrical energy is accumulated over time (integrated power)
% This function calculates instantaneous power; integration happens in main simulation
P_electrical_instantaneous = P_electrical;

%% Losses
% Mechanical losses (friction, gear mesh)
P_loss_mechanical = P_loss_sprocket + P_loss_gearbox + P_loss_bevel;

% Electrical losses (copper, core, etc.)
P_loss_electrical = P_losses_motor;

% Total losses
P_loss_total = P_loss_mechanical + P_loss_electrical;

%% Power Flow
% Mechanical power at drum
P_mechanical_drum = T_grav * omega_mass;

% Mechanical power at motor (reflected)
P_mechanical_motor = abs(T_grav * omega_mass / N_total);  % Power reaching motor

%% Energy Breakdown by Stage
energy_stages = struct();
energy_stages.drum = struct('P_in', P_mechanical_drum, 'P_loss', 0, 'P_out', P_mechanical_drum);
energy_stages.sprocket = struct('P_in', P_mechanical_drum, 'P_loss', P_loss_sprocket, ...
                                'P_out', P_mechanical_drum - P_loss_sprocket);
energy_stages.gearbox = struct('P_in', P_mechanical_drum - P_loss_sprocket, ...
                               'P_loss', P_loss_gearbox, ...
                               'P_out', P_mechanical_drum - P_loss_sprocket - P_loss_gearbox);
energy_stages.bevel = struct('P_in', P_mechanical_drum - P_loss_sprocket - P_loss_gearbox, ...
                             'P_loss', P_loss_bevel, ...
                             'P_out', P_mechanical_drum - P_loss_mechanical);
energy_stages.motor = struct('P_in', P_mechanical_drum - P_loss_mechanical, ...
                             'P_electrical', P_electrical_instantaneous, ...
                             'P_loss', P_loss_electrical);

%% Efficiency Metrics
% Stage efficiencies
eta_sprocket = 1 - (P_loss_sprocket / max(P_mechanical_drum, 1e-6));
eta_gearbox = 1 - (P_loss_gearbox / max(P_mechanical_drum - P_loss_sprocket, 1e-6));
eta_bevel = 1 - (P_loss_bevel / max(P_mechanical_drum - P_loss_sprocket - P_loss_gearbox, 1e-6));
eta_motor = P_electrical_instantaneous / max(P_mechanical_drum - P_loss_mechanical, 1e-6);

% Overall system efficiency
eta_system = P_electrical_instantaneous / max(P_mechanical_drum, 1e-6);

%% Assemble Output Structure
energy = struct();
energy.time = t;
energy.E_gravitational = E_gravitational;
energy.E_kinetic = E_kinetic;
energy.E_grav_initial = m * g * h_initial;
energy.P_mechanical = P_mechanical_drum;
energy.P_electrical = P_electrical_instantaneous;
energy.P_loss_mechanical = P_loss_mechanical;
energy.P_loss_electrical = P_loss_electrical;
energy.P_loss_total = P_loss_total;
energy.eta_system = eta_system;
energy.eta_sprocket = eta_sprocket;
energy.eta_gearbox = eta_gearbox;
energy.eta_bevel = eta_bevel;
energy.eta_motor = eta_motor;
energy.stages = energy_stages;
energy.I_motor = I_motor;
energy.V_terminal = V_terminal;
energy.omega_mass = omega_mass;
energy.omega_motor = omega_motor;
energy.h = h;

end
