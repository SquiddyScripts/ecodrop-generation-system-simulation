% SETUP_PATH Add all subdirectories to MATLAB path
%
% Run this script to ensure all functions are accessible.
% This script adds the directory containing this file and all subdirectories to the path.

% Get the directory where this script is located
script_path = fileparts(mfilename('fullpath'));

% If script_path is empty, fall back to pwd
if isempty(script_path)
    base_dir = pwd;
else
    base_dir = script_path;
end

% Add base directory and all subdirectories to path
addpath(genpath(base_dir));

% Alternatively, add specific subdirectories:
% addpath(fullfile(base_dir, 'models'));
% addpath(fullfile(base_dir, 'solvers'));
% addpath(fullfile(base_dir, 'analysis'));
% addpath(fullfile(base_dir, 'utils'));

fprintf('MATLAB path configured.\n');
fprintf('Added to path:\n');
fprintf('  - models/\n');
fprintf('  - solvers/\n');
fprintf('  - analysis/\n');
fprintf('  - utils/\n');
