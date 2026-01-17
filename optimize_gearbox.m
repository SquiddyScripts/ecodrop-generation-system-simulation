function opt_results = optimize_gearbox(params)
% OPTIMIZE_GEARBOX Perform continuous parameter sweep over gearbox ratios
%
% Sweeps gearbox ratio over a specified range and identifies optimal
% configuration based on efficiency, energy extracted, or other metrics.
%
% Inputs:
%   params - System parameters structure (from system_parameters.m)
%
% Outputs:
%   opt_results - Structure containing optimization results

%% Initialize
if nargin < 1
    params = system_parameters();
end

% Extract optimization parameters
N_min = params.optimization.gearbox_ratio_min;
N_max = params.optimization.gearbox_ratio_max;
N_step = params.optimization.gearbox_ratio_step;
metric = params.optimization.metric;

% Create gearbox ratio array
N_ratios = N_min:N_step:N_max;
n_ratios = length(N_ratios);

fprintf('Optimizing gearbox ratio from %.2f to %.2f (step: %.2f)...\n', ...
        N_min, N_max, N_step);
fprintf('Total ratios to test: %d\n\n', n_ratios);

%% Initialize Results Arrays
results_array = struct('ratio', cell(n_ratios, 1), ...
                       'E_electrical', [], ...
                       'E_losses', [], ...
                       'efficiency', [], ...
                       'drop_time', [], ...
                       'simulation', []);

%% Run Parameter Sweep
for i = 1:n_ratios
    N_current = N_ratios(i);
    
    fprintf('Testing ratio %.2f (%d/%d)... ', N_current, i, n_ratios);
    
    % Update gearbox ratio in params
    params.gearbox.ratios = [N_current];  % Single-stage gearbox
    
    % Run simulation
    try
        sim_results = main_simulation(params);
        
        % Extract metrics
        results_array(i).ratio = N_current;
        results_array(i).E_electrical = sim_results.final.E_electrical;
        results_array(i).E_losses = sim_results.final.E_losses;
        results_array(i).efficiency = sim_results.final.efficiency;
        results_array(i).drop_time = sim_results.final.drop_time;
        results_array(i).simulation = sim_results;
        
        fprintf('Efficiency: %.2f%%, Energy: %.2f J\n', ...
                100 * results_array(i).efficiency, results_array(i).E_electrical);
        
    catch ME
        warning('Simulation failed for ratio %.2f: %s', N_current, ME.message);
        results_array(i).ratio = N_current;
        results_array(i).E_electrical = 0;
        results_array(i).E_losses = inf;
        results_array(i).efficiency = 0;
        results_array(i).drop_time = inf;
        results_array(i).simulation = [];
    end
end

fprintf('\n');

%% Extract Data for Analysis
ratios = [results_array.ratio]';
E_electrical = [results_array.E_electrical]';
E_losses = [results_array.E_losses]';
efficiencies = [results_array.efficiency]';
drop_times = [results_array.drop_time]';

% Remove failed simulations
valid_idx = ~isinf(E_losses) & ~isnan(efficiencies);
ratios_valid = ratios(valid_idx);
E_electrical_valid = E_electrical(valid_idx);
E_losses_valid = E_losses(valid_idx);
efficiencies_valid = efficiencies(valid_idx);
drop_times_valid = drop_times(valid_idx);

%% Find Optimal Ratio
switch lower(metric)
    case 'efficiency'
        [opt_value, opt_idx] = max(efficiencies_valid);
        opt_ratio = ratios_valid(opt_idx);
    case 'energy'
        [opt_value, opt_idx] = max(E_electrical_valid);
        opt_ratio = ratios_valid(opt_idx);
    case 'power'
        % Power = Energy / Time
        power = E_electrical_valid ./ drop_times_valid;
        [opt_value, opt_idx] = max(power);
        opt_ratio = ratios_valid(opt_idx);
    otherwise
        warning('Unknown metric: %s. Using efficiency.', metric);
        [opt_value, opt_idx] = max(efficiencies_valid);
        opt_ratio = ratios_valid(opt_idx);
end

fprintf('Optimal gearbox ratio: %.2f\n', opt_ratio);
fprintf('Optimal %s: %.4f\n', metric, opt_value);
fprintf('Energy extracted at optimum: %.2f J\n', E_electrical_valid(opt_idx));
fprintf('Efficiency at optimum: %.2f%%\n', 100 * efficiencies_valid(opt_idx));

%% Statistical Analysis
% Find ratios within 1% of optimal efficiency
if strcmpi(metric, 'efficiency')
    threshold = 0.99 * opt_value;
    near_optimal_idx = efficiencies_valid >= threshold;
    near_optimal_ratios = ratios_valid(near_optimal_idx);
    
    fprintf('\nRatios within 1%% of optimal efficiency:\n');
    fprintf('  Range: %.2f to %.2f\n', min(near_optimal_ratios), max(near_optimal_ratios));
    fprintf('  Count: %d\n', sum(near_optimal_idx));
end

%% Assemble Output Structure
opt_results = struct();
opt_results.ratios = ratios;
opt_results.E_electrical = E_electrical;
opt_results.E_losses = E_losses;
opt_results.efficiencies = efficiencies;
opt_results.drop_times = drop_times;
opt_results.valid_idx = valid_idx;
opt_results.ratios_valid = ratios_valid;
opt_results.E_electrical_valid = E_electrical_valid;
opt_results.E_losses_valid = E_losses_valid;
opt_results.efficiencies_valid = efficiencies_valid;
opt_results.drop_times_valid = drop_times_valid;
opt_results.optimal_ratio = opt_ratio;
opt_results.optimal_value = opt_value;
opt_results.optimal_metric = metric;
opt_results.optimal_idx = opt_idx;
opt_results.all_results = results_array;
opt_results.params = params;

% Store optimal simulation results
if opt_idx <= length(results_array) && ~isempty(results_array(opt_idx).simulation)
    opt_results.optimal_simulation = results_array(opt_idx).simulation;
else
    opt_results.optimal_simulation = [];
end

end
