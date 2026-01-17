function results = main_simulation(params)
% MAIN_SIMULATION Run a single simulation of the electromechanical system
%
% Integrates the system ODE and returns time-series results including
% energy flow, losses, and all state variables.
%
% Inputs:
%   params - System parameters structure (from system_parameters.m)
%
% Outputs:
%   results - Structure containing all simulation results

%% Initialize Parameters
if nargin < 1
    params = system_parameters();
end

% Set initial conditions
h_initial = params.physical.initial_height;
r_drum = params.physical.drum_radius;
theta_initial = 0;  % Starting angular position
omega_initial = params.physical.initial_velocity / r_drum;  % Convert linear to angular

% Calculate initial motor speed (from kinematic constraint)
N_sprocket = params.physical.sprocket_ratio;
N_gearbox_total = prod(params.gearbox.ratios);
N_bevel = params.bevel.ratio;
N_total = N_sprocket * N_gearbox_total * N_bevel;
omega_motor_initial = omega_initial / N_total;

% Initial state vector
if params.simulation.include_electrical_dynamics
    % Estimate initial current
    V_emf_initial = params.motor.K_e * omega_motor_initial;
    I_initial = load_model(V_emf_initial, 0, params);
    state0 = [theta_initial; omega_initial; omega_motor_initial; I_initial];
else
    state0 = [theta_initial; omega_initial; omega_motor_initial];
end

params.simulation.initial_conditions = state0;

%% Setup ODE Solver
% Create wrapper function for ODE solver (MATLAB expects specific signature)
ode_wrapper = @(t, state) system_ode_wrapper(t, state, params);

% Solver options
options = odeset('RelTol', params.simulation.rel_tol, ...
                 'AbsTol', params.simulation.abs_tol, ...
                 'MaxStep', params.simulation.max_step, ...
                 'Events', @(t, state) mass_reached_bottom(t, state, params));

% Select solver
switch lower(params.simulation.solver)
    case 'ode45'
        solver = @ode45;
    case 'ode15s'
        solver = @ode15s;
    case 'ode23s'
        solver = @ode23s;
    otherwise
        solver = @ode45;
end

%% Run Simulation
fprintf('Running simulation...\n');
tic;
[t, state] = solver(ode_wrapper, params.simulation.t_span, state0, options);
sim_time = toc;
fprintf('Simulation completed in %.2f seconds.\n', sim_time);

%% Post-Process Results
% Calculate energy and other metrics at each time step
n_steps = length(t);
energy_history = cell(n_steps, 1);
component_history = cell(n_steps, 1);

fprintf('Post-processing results...\n');
for i = 1:n_steps
    % Call ODE function to get component outputs
    [~, comp_out] = system_ode(t(i), state(i,:)', params);
    
    if ~isempty(comp_out)
        component_history{i} = comp_out;
        energy_history{i} = energy_analysis(t(i), state(i,:)', params, comp_out);
    end
end

%% Integrate Energy Over Time
% Calculate cumulative electrical energy
E_electrical_cumulative = zeros(n_steps, 1);
E_loss_mechanical_cumulative = zeros(n_steps, 1);
E_loss_electrical_cumulative = zeros(n_steps, 1);

for i = 2:n_steps
    dt = t(i) - t(i-1);
    if ~isempty(energy_history{i}) && ~isempty(energy_history{i-1})
        P_elec_avg = (energy_history{i}.P_electrical + energy_history{i-1}.P_electrical) / 2;
        P_loss_mech_avg = (energy_history{i}.P_loss_mechanical + energy_history{i-1}.P_loss_mechanical) / 2;
        P_loss_elec_avg = (energy_history{i}.P_loss_electrical + energy_history{i-1}.P_loss_electrical) / 2;
        
        E_electrical_cumulative(i) = E_electrical_cumulative(i-1) + P_elec_avg * dt;
        E_loss_mechanical_cumulative(i) = E_loss_mechanical_cumulative(i-1) + P_loss_mech_avg * dt;
        E_loss_electrical_cumulative(i) = E_loss_electrical_cumulative(i-1) + P_loss_elec_avg * dt;
    end
end

%% Extract Time-Series Data
theta_mass = state(:, 1);
omega_mass = state(:, 2);
omega_motor = state(:, 3);

if params.simulation.include_electrical_dynamics && size(state, 2) >= 4
    I_motor = state(:, 4);
else
    % Calculate current from load model
    I_motor = zeros(n_steps, 1);
    V_terminal = zeros(n_steps, 1);
    for i = 1:n_steps
        if ~isempty(component_history{i})
            I_motor(i) = component_history{i}.I_motor;
            V_terminal(i) = component_history{i}.V_terminal;
        end
    end
end

% Extract other quantities
h = zeros(n_steps, 1);
v_linear = zeros(n_steps, 1);
P_mechanical = zeros(n_steps, 1);
P_electrical = zeros(n_steps, 1);
eta_system = zeros(n_steps, 1);

for i = 1:n_steps
    if ~isempty(energy_history{i})
        h(i) = energy_history{i}.h;
        v_linear(i) = r_drum * omega_mass(i);
        P_mechanical(i) = energy_history{i}.P_mechanical;
        P_electrical(i) = energy_history{i}.P_electrical;
        eta_system(i) = energy_history{i}.eta_system;
    end
end

%% Energy Conservation Check
E_grav_initial = params.physical.mass * params.physical.g * h_initial;
E_kinetic_final = 0.5 * params.physical.mass * v_linear(end)^2;  % Simplified
E_electrical_final = E_electrical_cumulative(end);
E_losses_final = E_loss_mechanical_cumulative(end) + E_loss_electrical_cumulative(end);

energy_error = abs(E_grav_initial - (E_kinetic_final + E_electrical_final + E_losses_final));
energy_error_percent = 100 * energy_error / E_grav_initial;

fprintf('Energy conservation check:\n');
fprintf('  Initial gravitational energy: %.2f J\n', E_grav_initial);
fprintf('  Final kinetic energy: %.2f J\n', E_kinetic_final);
fprintf('  Final electrical energy: %.2f J\n', E_electrical_final);
fprintf('  Final losses: %.2f J\n', E_losses_final);
fprintf('  Energy error: %.2f J (%.2f%%)\n', energy_error, energy_error_percent);

%% Validation Checks (will be called after results structure is created)

%% Assemble Results Structure
results = struct();
results.time = t;
results.state = state;
results.theta_mass = theta_mass;
results.omega_mass = omega_mass;
results.omega_motor = omega_motor;
results.I_motor = I_motor;
results.h = h;
results.v_linear = v_linear;
results.P_mechanical = P_mechanical;
results.P_electrical = P_electrical;
results.E_electrical_cumulative = E_electrical_cumulative;
results.E_loss_mechanical_cumulative = E_loss_mechanical_cumulative;
results.E_loss_electrical_cumulative = E_loss_electrical_cumulative;
results.eta_system = eta_system;
results.energy_history = energy_history;
results.component_history = component_history;
results.energy_conservation = struct('error', energy_error, 'error_percent', energy_error_percent);
results.params = params;
results.sim_time = sim_time;

% Final metrics
results.final = struct();
results.final.E_electrical = E_electrical_final;
results.final.E_losses = E_losses_final;
results.final.efficiency = E_electrical_final / E_grav_initial;
results.final.drop_time = t(end);
results.final.final_height = h(end);

%% Run Validation Checks
results.validation = validation_checks(results, params);

end

%% Helper Functions

function dstate = system_ode_wrapper(t, state, params)
% Wrapper for ODE solver (MATLAB expects specific signature)
[dstate, ~] = system_ode(t, state, params);
end

function [value, isterminal, direction] = mass_reached_bottom(t, state, params)
% Event function to stop simulation when mass reaches bottom
theta = state(1);
r_drum = params.physical.drum_radius;
h_initial = params.physical.initial_height;
h = h_initial - r_drum * theta;

value = h;  % Event occurs when h = 0
isterminal = params.simulation.stop_at_bottom;  % Stop integration
direction = -1;  % Detect when crossing zero from above
end
