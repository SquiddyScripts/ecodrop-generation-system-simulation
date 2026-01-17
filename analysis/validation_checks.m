function validation = validation_checks(results, params)
% VALIDATION_CHECKS Perform validation and verification checks
%
% Checks energy conservation, unit consistency, and compares with
% analytical solutions where applicable.
%
% Inputs:
%   results - Simulation results structure
%   params - System parameters structure
%
% Outputs:
%   validation - Structure containing validation results

validation = struct();
validation.passed = true;
validation.warnings = {};
validation.errors = {};

%% Energy Conservation Check
E_grav_initial = params.physical.mass * params.physical.g * params.physical.initial_height;
E_elec_final = results.E_electrical_cumulative(end);
E_loss_mech_final = results.E_loss_mechanical_cumulative(end);
E_loss_elec_final = results.E_loss_electrical_cumulative(end);
E_kin_final = 0.5 * params.physical.mass * results.v_linear(end)^2;

E_total_final = E_elec_final + E_loss_mech_final + E_loss_elec_final + E_kin_final;
E_error = abs(E_grav_initial - E_total_final);
E_error_percent = 100 * E_error / E_grav_initial;

validation.energy_conservation = struct();
validation.energy_conservation.E_initial = E_grav_initial;
validation.energy_conservation.E_final = E_total_final;
validation.energy_conservation.error = E_error;
validation.energy_conservation.error_percent = E_error_percent;

% Tolerance: 1% error is acceptable for numerical integration
if E_error_percent > 1.0
    validation.passed = false;
    validation.errors{end+1} = sprintf('Energy conservation error too large: %.2f%%', E_error_percent);
elseif E_error_percent > 0.1
    validation.warnings{end+1} = sprintf('Energy conservation error: %.2f%% (acceptable but high)', E_error_percent);
else
    fprintf('✓ Energy conservation check passed (error: %.4f%%)\n', E_error_percent);
end

%% Unit Consistency Check
validation.unit_consistency = struct();
validation.unit_consistency.passed = true;

% Check that all quantities have reasonable magnitudes
% Mass should be in kg (typically 1-1000 kg)
if params.physical.mass < 0.001 || params.physical.mass > 10000
    validation.unit_consistency.passed = false;
    validation.errors{end+1} = sprintf('Mass value suspicious: %.2e kg', params.physical.mass);
end

% Velocity should be in m/s (typically 0-100 m/s for this system)
if max(abs(results.v_linear)) > 1000
    validation.unit_consistency.passed = false;
    validation.errors{end+1} = sprintf('Velocity too high: %.2f m/s (check units)', max(abs(results.v_linear)));
end

% Current should be in A (typically 0-100 A for this motor)
if max(abs(results.I_motor)) > 1000
    validation.unit_consistency.passed = false;
    validation.errors{end+1} = sprintf('Current too high: %.2f A (check units)', max(abs(results.I_motor)));
end

% Voltage should be in V (typically 0-100 V for this system)
V_max = 0;
for i = 1:length(results.component_history)
    if ~isempty(results.component_history{i})
        V_max = max(V_max, abs(results.component_history{i}.V_terminal));
    end
end
if V_max > 1000
    validation.unit_consistency.passed = false;
    validation.errors{end+1} = sprintf('Voltage too high: %.2f V (check units)', V_max);
end

if validation.unit_consistency.passed
    fprintf('✓ Unit consistency check passed\n');
end

%% Physical Reasonableness Checks
validation.physical_reasonableness = struct();
validation.physical_reasonableness.passed = true;

% Check that mass doesn't fall faster than free fall
v_free_fall = sqrt(2 * params.physical.g * params.physical.initial_height);
v_max_sim = max(results.v_linear);
if v_max_sim > v_free_fall * 1.1  % Allow 10% tolerance
    validation.physical_reasonableness.passed = false;
    validation.errors{end+1} = sprintf('Mass falling faster than free fall: %.2f > %.2f m/s', ...
                                       v_max_sim, v_free_fall);
else
    fprintf('✓ Physical reasonableness check passed (max velocity: %.2f m/s)\n', v_max_sim);
end

% Check that efficiency is between 0 and 1
eta_final = results.eta_system(end);
if eta_final < 0 || eta_final > 1.1  % Allow slight overshoot due to numerical errors
    validation.physical_reasonableness.passed = false;
    validation.errors{end+1} = sprintf('Efficiency out of bounds: %.4f', eta_final);
end

% Check that motor current doesn't exceed maximum
I_max_sim = max(results.I_motor);
if I_max_sim > params.motor.I_max * 1.1  % Allow 10% tolerance
    validation.warnings{end+1} = sprintf('Motor current exceeds rated maximum: %.2f > %.2f A', ...
                                        I_max_sim, params.motor.I_max);
end

%% Numerical Stability Check
validation.numerical_stability = struct();
validation.numerical_stability.passed = true;

% Check for NaN or Inf values
if any(isnan(results.state(:))) || any(isinf(results.state(:)))
    validation.numerical_stability.passed = false;
    validation.errors{end+1} = 'NaN or Inf values detected in state vector';
else
    fprintf('✓ Numerical stability check passed\n');
end

% Check for sudden jumps (indicating numerical instability)
domega_dt = diff(results.omega_mass) ./ diff(results.time);
if any(abs(domega_dt) > 1000)  % Arbitrary threshold
    validation.warnings{end+1} = 'Large acceleration spikes detected (possible numerical instability)';
end

%% Summary
fprintf('\n=== Validation Summary ===\n');
if validation.passed && isempty(validation.errors)
    fprintf('✓ All critical checks passed\n');
else
    fprintf('✗ Some checks failed\n');
end

if ~isempty(validation.errors)
    fprintf('\nErrors:\n');
    for i = 1:length(validation.errors)
        fprintf('  ✗ %s\n', validation.errors{i});
    end
end

if ~isempty(validation.warnings)
    fprintf('\nWarnings:\n');
    for i = 1:length(validation.warnings)
        fprintf('  ⚠ %s\n', validation.warnings{i});
    end
end

fprintf('========================\n\n');

end
