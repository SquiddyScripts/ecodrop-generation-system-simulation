function N_total = calculate_total_ratio(ratios)
% CALCULATE_TOTAL_RATIO Calculate total gear ratio from stage ratios
%
% For a multi-stage gearbox, the total ratio is the product of
% individual stage ratios.
%
% Inputs:
%   ratios - Array of gear ratios for each stage
%
% Outputs:
%   N_total - Total gear ratio

if isempty(ratios)
    N_total = 1.0;
    return;
end

N_total = prod(ratios);

end

function N_cumulative = calculate_cumulative_ratios(ratios)
% CALCULATE_CUMULATIVE_RATIOS Calculate cumulative ratios for each stage
%
% Returns an array where each element is the cumulative ratio up to
% that stage. Useful for reflecting inertias through multi-stage gearboxes.
%
% Inputs:
%   ratios - Array of gear ratios for each stage
%
% Outputs:
%   N_cumulative - Array of cumulative ratios

n_stages = length(ratios);
N_cumulative = zeros(1, n_stages);

if n_stages == 0
    return;
end

N_cumulative(1) = ratios(1);
for i = 2:n_stages
    N_cumulative(i) = N_cumulative(i-1) * ratios(i);
end

end
