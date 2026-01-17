function plot_results(results, opt_results)
% PLOT_RESULTS Create comprehensive visualization of simulation results
%
% Generates time-series plots, energy analysis charts, and optimization
% result visualizations.
%
% Inputs:
%   results - Simulation results structure (from main_simulation.m)
%   opt_results - Optional optimization results (from optimize_gearbox.m)

if nargin < 2
    opt_results = [];
end

%% Time-Series Plots
if ~isempty(results)
    plot_time_series(results);
end

%% Energy Analysis
if ~isempty(results)
    plot_energy_analysis(results);
end

%% Optimization Results
if ~isempty(opt_results)
    plot_optimization_results(opt_results);
end

end

function plot_time_series(results)
% Plot time-series data

figure('Name', 'Time-Series Results', 'Position', [100, 100, 1200, 800]);

% Mass position and velocity
subplot(3, 3, 1);
plot(results.time, results.h, 'b-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Height [m]');
title('Mass Height vs. Time');
grid on;

subplot(3, 3, 2);
plot(results.time, results.v_linear, 'r-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Mass Velocity vs. Time');
grid on;

% Angular velocities
subplot(3, 3, 3);
yyaxis left;
plot(results.time, results.omega_mass, 'b-', 'LineWidth', 2);
ylabel('Drum Angular Velocity [rad/s]');
yyaxis right;
plot(results.time, results.omega_motor, 'r-', 'LineWidth', 2);
ylabel('Motor Angular Velocity [rad/s]');
xlabel('Time [s]');
title('Angular Velocities vs. Time');
legend('Drum', 'Motor', 'Location', 'best');
grid on;

% Electrical quantities
subplot(3, 3, 4);
yyaxis left;
plot(results.time, results.I_motor, 'b-', 'LineWidth', 2);
ylabel('Current [A]');
yyaxis right;
% Extract voltage from component history
V_terminal = zeros(size(results.time));
for i = 1:length(results.time)
    if ~isempty(results.component_history{i})
        V_terminal(i) = results.component_history{i}.V_terminal;
    end
end
plot(results.time, V_terminal, 'r-', 'LineWidth', 2);
ylabel('Voltage [V]');
xlabel('Time [s]');
title('Motor Current and Voltage');
legend('Current', 'Voltage', 'Location', 'best');
grid on;

% Power
subplot(3, 3, 5);
yyaxis left;
plot(results.time, results.P_mechanical, 'b-', 'LineWidth', 2);
ylabel('Mechanical Power [W]');
yyaxis right;
plot(results.time, results.P_electrical, 'r-', 'LineWidth', 2);
ylabel('Electrical Power [W]');
xlabel('Time [s]');
title('Power vs. Time');
legend('Mechanical', 'Electrical', 'Location', 'best');
grid on;

% Efficiency
subplot(3, 3, 6);
plot(results.time, 100 * results.eta_system, 'g-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Efficiency [%]');
title('System Efficiency vs. Time');
grid on;

% Energy (cumulative)
subplot(3, 3, 7);
plot(results.time, results.E_electrical_cumulative, 'b-', 'LineWidth', 2);
hold on;
plot(results.time, results.E_loss_mechanical_cumulative, 'r-', 'LineWidth', 2);
plot(results.time, results.E_loss_electrical_cumulative, 'm-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Energy [J]');
title('Cumulative Energy');
legend('Electrical', 'Mechanical Losses', 'Electrical Losses', 'Location', 'best');
grid on;
hold off;

% Energy breakdown (pie chart at final time)
subplot(3, 3, 8);
E_grav_initial = results.params.physical.mass * results.params.physical.g * ...
                 results.params.physical.initial_height;
E_elec_final = results.E_electrical_cumulative(end);
E_loss_mech_final = results.E_loss_mechanical_cumulative(end);
E_loss_elec_final = results.E_loss_electrical_cumulative(end);
E_kin_final = 0.5 * results.params.physical.mass * results.v_linear(end)^2;

% Remaining energy (if any)
E_remaining = E_grav_initial - E_elec_final - E_loss_mech_final - E_loss_elec_final - E_kin_final;
E_remaining = max(0, E_remaining);

pie_data = [E_elec_final, E_loss_mech_final, E_loss_elec_final, E_kin_final, E_remaining];
pie_labels = {'Electrical', 'Mech. Losses', 'Elec. Losses', 'Kinetic', 'Remaining'};
pie(pie_data, pie_labels);
title('Final Energy Distribution');

% State space (omega_mass vs. theta)
subplot(3, 3, 9);
plot(results.theta_mass, results.omega_mass, 'b-', 'LineWidth', 2);
xlabel('Angular Position [rad]');
ylabel('Angular Velocity [rad/s]');
title('Phase Portrait (Drum)');
grid on;

sgtitle('Time-Series Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');

end

function plot_energy_analysis(results)
% Plot detailed energy analysis

figure('Name', 'Energy Analysis', 'Position', [150, 150, 1000, 600]);

% Energy vs. time
subplot(2, 2, 1);
E_grav_initial = results.params.physical.mass * results.params.physical.g * ...
                 results.params.physical.initial_height;
E_grav = E_grav_initial - results.params.physical.mass * results.params.physical.g * ...
         (results.params.physical.initial_height - results.h);

plot(results.time, E_grav, 'b-', 'LineWidth', 2);
hold on;
plot(results.time, results.E_electrical_cumulative, 'g-', 'LineWidth', 2);
plot(results.time, results.E_loss_mechanical_cumulative + results.E_loss_electrical_cumulative, ...
     'r-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Energy [J]');
title('Energy Flow vs. Time');
legend('Gravitational', 'Electrical', 'Total Losses', 'Location', 'best');
grid on;
hold off;

% Power breakdown by stage
subplot(2, 2, 2);
if ~isempty(results.energy_history) && ~isempty(results.energy_history{end})
    stages = results.energy_history{end}.stages;
    stage_names = fieldnames(stages);
    P_losses = zeros(length(stage_names), 1);
    for i = 1:length(stage_names)
        stage = stages.(stage_names{i});
        if isfield(stage, 'P_loss')
            P_losses(i) = stage.P_loss;
        end
    end
    bar(P_losses);
    set(gca, 'XTickLabel', stage_names);
    xlabel('Stage');
    ylabel('Power Loss [W]');
    title('Power Loss by Stage (Final)');
    grid on;
end

% Efficiency by stage
subplot(2, 2, 3);
if ~isempty(results.energy_history) && ~isempty(results.energy_history{end})
    eta_sprocket = results.energy_history{end}.eta_sprocket;
    eta_gearbox = results.energy_history{end}.eta_gearbox;
    eta_bevel = results.energy_history{end}.eta_bevel;
    eta_motor = results.energy_history{end}.eta_motor;
    
    eta_data = [eta_sprocket, eta_gearbox, eta_bevel, eta_motor] * 100;
    eta_names = {'Sprocket', 'Gearbox', 'Bevel', 'Motor'};
    bar(eta_data);
    set(gca, 'XTickLabel', eta_names);
    ylabel('Efficiency [%]');
    title('Stage Efficiencies (Final)');
    ylim([0, 100]);
    grid on;
end

% Energy Sankey-style visualization (simplified bar chart)
subplot(2, 2, 4);
E_grav_initial = results.params.physical.mass * results.params.physical.g * ...
                 results.params.physical.initial_height;
E_elec_final = results.E_electrical_cumulative(end);
E_loss_total = results.E_loss_mechanical_cumulative(end) + ...
               results.E_loss_electrical_cumulative(end);
E_kin_final = 0.5 * results.params.physical.mass * results.v_linear(end)^2;

energy_flow = [E_grav_initial, E_elec_final, E_loss_total, E_kin_final];
energy_labels = {'Initial\nGrav.', 'Electrical', 'Losses', 'Kinetic'};
bar(energy_flow);
set(gca, 'XTickLabel', energy_labels);
ylabel('Energy [J]');
title('Energy Flow Summary');
grid on;

sgtitle('Energy Analysis', 'FontSize', 14, 'FontWeight', 'bold');

end

function plot_optimization_results(opt_results)
% Plot optimization results

figure('Name', 'Gearbox Optimization Results', 'Position', [200, 200, 1200, 800]);

% Efficiency vs. gearbox ratio
subplot(2, 3, 1);
plot(opt_results.ratios_valid, 100 * opt_results.efficiencies_valid, 'b-', 'LineWidth', 2);
hold on;
plot(opt_results.optimal_ratio, 100 * opt_results.optimal_value, 'ro', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Gearbox Ratio');
ylabel('Efficiency [%]');
title('Efficiency vs. Gearbox Ratio');
legend('Efficiency', 'Optimal', 'Location', 'best');
grid on;
hold off;

% Energy extracted vs. gearbox ratio
subplot(2, 3, 2);
plot(opt_results.ratios_valid, opt_results.E_electrical_valid, 'g-', 'LineWidth', 2);
hold on;
plot(opt_results.optimal_ratio, opt_results.E_electrical_valid(opt_results.optimal_idx), ...
     'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Gearbox Ratio');
ylabel('Electrical Energy [J]');
title('Energy Extracted vs. Gearbox Ratio');
legend('Energy', 'Optimal', 'Location', 'best');
grid on;
hold off;

% Losses vs. gearbox ratio
subplot(2, 3, 3);
plot(opt_results.ratios_valid, opt_results.E_losses_valid, 'r-', 'LineWidth', 2);
xlabel('Gearbox Ratio');
ylabel('Total Losses [J]');
title('Losses vs. Gearbox Ratio');
grid on;

% Drop time vs. gearbox ratio
subplot(2, 3, 4);
plot(opt_results.ratios_valid, opt_results.drop_times_valid, 'm-', 'LineWidth', 2);
hold on;
plot(opt_results.optimal_ratio, opt_results.drop_times_valid(opt_results.optimal_idx), ...
     'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Gearbox Ratio');
ylabel('Drop Time [s]');
title('Drop Time vs. Gearbox Ratio');
legend('Drop Time', 'Optimal', 'Location', 'best');
grid on;
hold off;

% Energy breakdown at optimal ratio
subplot(2, 3, 5);
if ~isempty(opt_results.optimal_simulation)
    E_grav_initial = opt_results.params.physical.mass * opt_results.params.physical.g * ...
                     opt_results.params.physical.initial_height;
    E_elec = opt_results.optimal_simulation.final.E_electrical;
    E_losses = opt_results.optimal_simulation.final.E_losses;
    E_kin = 0.5 * opt_results.params.physical.mass * ...
            opt_results.optimal_simulation.v_linear(end)^2;
    E_remaining = E_grav_initial - E_elec - E_losses - E_kin;
    E_remaining = max(0, E_remaining);
    
    pie_data = [E_elec, E_losses, E_kin, E_remaining];
    pie_labels = {'Electrical', 'Losses', 'Kinetic', 'Remaining'};
    pie(pie_data, pie_labels);
    title(sprintf('Energy at Optimal Ratio (%.2f)', opt_results.optimal_ratio));
end

% Combined metric plot
subplot(2, 3, 6);
yyaxis left;
plot(opt_results.ratios_valid, 100 * opt_results.efficiencies_valid, 'b-', 'LineWidth', 2);
ylabel('Efficiency [%]');
yyaxis right;
plot(opt_results.ratios_valid, opt_results.E_electrical_valid, 'g-', 'LineWidth', 2);
ylabel('Energy [J]');
xlabel('Gearbox Ratio');
title('Efficiency and Energy vs. Ratio');
hold on;
xline(opt_results.optimal_ratio, 'r--', 'LineWidth', 2, 'DisplayName', 'Optimal');
legend('Efficiency', 'Energy', 'Optimal', 'Location', 'best');
grid on;
hold off;

sgtitle('Gearbox Optimization Results', 'FontSize', 14, 'FontWeight', 'bold');

% Print summary
fprintf('\n=== Optimization Summary ===\n');
fprintf('Optimal gearbox ratio: %.2f\n', opt_results.optimal_ratio);
fprintf('Optimal efficiency: %.2f%%\n', 100 * opt_results.optimal_value);
fprintf('Energy extracted: %.2f J\n', opt_results.E_electrical_valid(opt_results.optimal_idx));
fprintf('Drop time: %.2f s\n', opt_results.drop_times_valid(opt_results.optimal_idx));
fprintf('===========================\n\n');

end
