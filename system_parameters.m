function params = system_parameters()
% SYSTEM_PARAMETERS Define all physical, motor, loss, and simulation parameters
%
% Returns a structure containing all parameters for the electromechanical
% simulation system. All units are SI (kg, m, s, N, N·m, W, J, Ω, V, A).
%
% Usage:
%   params = system_parameters();
%
% Output:
%   params - Structure containing all system parameters

%% Physical System Parameters
params.physical = struct();
params.physical.mass = 10.0;              % Mass of falling object [kg]
params.physical.g = 9.81;                 % Gravitational acceleration [m/s²]
params.physical.drum_radius = 0.05;     % Drum radius [m]
params.physical.drum_inertia = 0.01;      % Drum moment of inertia [kg·m²]
params.physical.sprocket_ratio = 1.0;     % Sprocket speed ratio (output/input)
params.physical.sprocket_inertia = 0.005; % Sprocket moment of inertia [kg·m²]
params.physical.initial_height = 10.0;    % Initial drop height [m]
params.physical.initial_velocity = 0.0;   % Initial velocity [m/s]

%% Gearbox Parameters
params.gearbox = struct();
params.gearbox.ratios = [5.0];            % Gearbox ratios (can be multi-stage array)
params.gearbox.inertia_input = 0.002;     % Input shaft inertia [kg·m²]
params.gearbox.inertia_output = 0.001;    % Output shaft inertia [kg·m²]
params.gearbox.inertia_intermediate = [];  % Intermediate shaft inertias [kg·m²] (empty if single stage)

%% Bevel Gear Parameters
params.bevel = struct();
params.bevel.ratio = 1.0;                 % Bevel gear ratio (1:1, axis redirection only)
params.bevel.inertia = 0.001;             % Bevel gear inertia [kg·m²]

%% Motor Parameters (Turnigy Aerodrive SK3-5065-236KV)
params.motor = struct();
params.motor.Kv = 236;                    % Speed constant [RPM/V]
params.motor.K_e = 60 / (2 * pi * params.motor.Kv);  % Back-EMF constant [V·s/rad]
params.motor.K_t = params.motor.K_e;      % Torque constant [N·m/A] (ideal motor: K_t = K_e)
params.motor.R_winding = 0.019;           % Winding resistance [Ω]
params.motor.L_winding = 0.0001;          % Winding inductance [H] (optional, for dynamics)
params.motor.J_rotor = 0.001;             % Rotor moment of inertia [kg·m²] (parameterized)
params.motor.I_max = 60.0;                % Maximum current [A]
params.motor.core_loss_coefficient = 0.0; % Core loss coefficient [W/(rad/s)²] (optional)
params.motor.mechanical_loss_coefficient = 0.0; % Mechanical loss coefficient [N·m·s/rad] (optional)

%% Loss Parameters (Hybrid Approach)
params.losses = struct();

% Efficiency-based losses (for gear stages)
params.losses.drum_efficiency = 0.98;     % Drum bearing efficiency
params.losses.sprocket_efficiency = 0.95;  % Sprocket efficiency
params.losses.gearbox_efficiency = 0.92;   % Gearbox efficiency (per stage or overall)
params.losses.bevel_efficiency = 0.97;     % Bevel gear efficiency

% Detailed friction losses (Coulomb + Viscous)
params.losses.drum_coulomb = 0.1;         % Drum Coulomb friction torque [N·m]
params.losses.drum_viscous = 0.01;        % Drum viscous damping [N·m·s/rad]

params.losses.sprocket_coulomb = 0.05;    % Sprocket Coulomb friction [N·m]
params.losses.sprocket_viscous = 0.005;   % Sprocket viscous damping [N·m·s/rad]

params.losses.gearbox_coulomb = 0.02;     % Gearbox Coulomb friction [N·m]
params.losses.gearbox_viscous = 0.002;    % Gearbox viscous damping [N·m·s/rad]

params.losses.bevel_coulomb = 0.01;       % Bevel gear Coulomb friction [N·m]
params.losses.bevel_viscous = 0.001;      % Bevel gear viscous damping [N·m·s/rad]

params.losses.motor_bearing_coulomb = 0.005;  % Motor bearing Coulomb friction [N·m]
params.losses.motor_bearing_viscous = 0.0005; % Motor bearing viscous damping [N·m·s/rad]

%% Electrical Load Parameters
params.electrical = struct();
params.electrical.load_type = 'resistive';  % Load type: 'resistive', 'battery', 'converter', 'custom'
params.electrical.R_load = 1.0;             % Load resistance [Ω] (for resistive load)
params.electrical.load_function = [];        % Custom load function handle (if load_type = 'custom')
params.electrical.load_params = struct();   % Additional load parameters

% Battery load parameters (if load_type = 'battery')
params.electrical.load_params.V_battery = 12.0;      % Battery voltage [V]
params.electrical.load_params.R_battery = 0.1;      % Battery internal resistance [Ω]
params.electrical.load_params.I_max_charge = 30.0;   % Maximum charge current [A]

% Power converter parameters (if load_type = 'converter')
params.electrical.load_params.P_demand = 100.0;      % Power demand [W]
params.electrical.load_params.converter_efficiency = 0.95; % Converter efficiency

%% Simulation Parameters
params.simulation = struct();
params.simulation.t_span = [0, 10.0];      % Time span [s]
params.simulation.initial_conditions = []; % Will be set based on initial_height
params.simulation.solver = 'ode45';        % ODE solver: 'ode45', 'ode15s', 'ode23s'
params.simulation.rel_tol = 1e-6;         % Relative tolerance
params.simulation.abs_tol = 1e-9;         % Absolute tolerance
params.simulation.max_step = 0.01;        % Maximum step size [s]
params.simulation.include_electrical_dynamics = false; % Include I as state variable

% Termination conditions
params.simulation.stop_at_bottom = true;   % Stop simulation when mass reaches bottom
params.simulation.stop_at_rest = false;   % Stop simulation when system comes to rest
params.simulation.min_velocity = 1e-6;    % Minimum velocity for "at rest" [m/s]

%% Energy Tracking Parameters
params.energy = struct();
params.energy.track_detailed = true;       % Track detailed energy breakdown
params.energy.track_stage_losses = true;  % Track losses per stage

%% Optimization Parameters (for gearbox optimization)
params.optimization = struct();
params.optimization.gearbox_ratio_min = 1.0;   % Minimum gearbox ratio
params.optimization.gearbox_ratio_max = 50.0; % Maximum gearbox ratio
params.optimization.gearbox_ratio_step = 0.5; % Step size for ratio sweep
params.optimization.metric = 'efficiency';    % Optimization metric: 'efficiency', 'energy', 'power'

end
