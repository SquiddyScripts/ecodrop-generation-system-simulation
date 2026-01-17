% RUN_EXAMPLE Example script to run simulation and optimization
%
% This script demonstrates how to use the electromechanical simulation
% system to run a single simulation and perform gearbox optimization.

clear; close all; clc;

fprintf('=== Electromechanical System Simulation Example ===\n\n');

% Setup MATLAB path
setup_path;

%% Load Parameters
params = system_parameters();

% Optionally modify parameters
% params.physical.mass = 20.0;  % [kg]
% params.physical.initial_height = 15.0;  % [m]
% params.electrical.R_load = 0.5;  % [Ω]

%% Run Single Simulation
fprintf('Running single simulation...\n');
results = main_simulation(params);

% Display key results
fprintf('\n=== Simulation Results ===\n');
fprintf('Drop time: %.2f s\n', results.final.drop_time);
fprintf('Electrical energy extracted: %.2f J\n', results.final.E_electrical);
fprintf('System efficiency: %.2f%%\n', 100 * results.final.efficiency);
fprintf('Final height: %.2f m\n', results.final.final_height);
fprintf('==========================\n\n');

%% Plot Results
fprintf('Generating plots...\n');
plot_results(results);

%% Run Gearbox Optimization
fprintf('\nRunning gearbox optimization...\n');
opt_results = optimize_gearbox(params);

% Plot optimization results
plot_results([], opt_results);

fprintf('\n=== Example Complete ===\n');
