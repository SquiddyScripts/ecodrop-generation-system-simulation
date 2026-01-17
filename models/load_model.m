function I = load_model(V_terminal, t, params)
% LOAD_MODEL Abstracted electrical load model
%
% Determines load current based on terminal voltage and load type.
% Supports resistive, battery, converter, and custom load models.
%
% Inputs:
%   V_terminal - Terminal voltage [V]
%   t - Current time [s]
%   params - System parameters structure (contains electrical.load_*)
%
% Outputs:
%   I - Load current [A]

load_type = params.electrical.load_type;
load_params = params.electrical.load_params;

switch lower(load_type)
    case 'resistive'
        % Simple resistive load
        R_load = params.electrical.R_load;
        if R_load > 0
            I = V_terminal / R_load;
        else
            I = 0;
        end
        
    case 'battery'
        % Battery charging model
        V_battery = load_params.V_battery;
        R_battery = load_params.R_battery;
        I_max = load_params.I_max_charge;
        
        % Current limited by battery voltage and internal resistance
        if V_terminal > V_battery
            I = (V_terminal - V_battery) / (R_battery + 1e-6);  % Small resistance to avoid division by zero
        else
            I = 0;  % No charging if terminal voltage below battery voltage
        end
        
        % Apply current limit
        I = min(I, I_max);
        I = max(I, 0);  % No negative current (no discharging)
        
    case 'converter'
        % Power converter model (constant power load)
        P_demand = load_params.P_demand;
        eta_conv = load_params.converter_efficiency;
        
        if V_terminal > 0.1  % Minimum voltage threshold
            % Power demand at input (accounting for converter efficiency)
            P_input = P_demand / eta_conv;
            I = P_input / V_terminal;
        else
            I = 0;
        end
        
    case 'custom'
        % User-defined load function
        if ~isempty(params.electrical.load_function)
            I = params.electrical.load_function(V_terminal, t, load_params);
        else
            warning('Custom load type specified but no load_function provided. Using resistive load.');
            R_load = params.electrical.R_load;
            I = V_terminal / max(R_load, 1e-6);
        end
        
    otherwise
        error('Unknown load type: %s', load_type);
end

% Ensure current is non-negative (generator mode)
I = max(I, 0);

end
