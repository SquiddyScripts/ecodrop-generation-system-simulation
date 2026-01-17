# Electromechanical System Simulation

A physics-accurate, time-domain electromechanical simulation of a gravity-driven generator system. The simulation models a falling mass driving a motor/generator through a multi-stage drivetrain (drum → sprocket → gearbox → bevel gears → motor).

## System Overview

The simulation models:
- **Falling mass** attached to a rope wrapped around a drum
- **Drum** driving a sprocket
- **Sprocket** driving a gearbox
- **Gearbox** output connecting through 1:1 bevel gears (axis redirection)
- **DC/BLDC motor** (Turnigy Aerodrive SK3-5065-236KV) operating in generator mode

## Features

- **Time-domain dynamics**: Full transient simulation, not steady-state
- **Energy flow tracking**: Gravitational → kinetic → electrical energy conversion
- **Loss modeling**: Hybrid approach (efficiency factors + detailed friction)
- **Modular design**: Each component modeled separately
- **Gearbox optimization**: Continuous parameter sweep to find optimal ratio
- **Professional-grade**: System-level simulation suitable for engineering analysis

## File Structure

```
qog/
├── system_parameters.m          % Parameter definition
├── main_simulation.m             % Single simulation run
├── optimize_gearbox.m           % Gearbox optimization
├── run_example.m                % Example usage script
├── models/
│   ├── mass_drum_model.m        % Mass and drum dynamics
│   ├── sprocket_model.m         % Sprocket transformation
│   ├── gearbox_model.m           % Gearbox module
│   ├── bevel_gear_model.m       % Bevel gear (1:1)
│   ├── motor_model.m            % SK3 motor/generator
│   └── load_model.m             % Electrical load abstraction
├── solvers/
│   └── system_ode.m              % ODE function
├── analysis/
│   ├── energy_analysis.m         % Energy flow calculations
│   ├── plot_results.m            % Visualization functions
│   └── validation_checks.m      % Validation and verification
└── utils/
    ├── reflect_inertia.m         % Inertia reflection
    └── gear_ratios.m             % Gear ratio calculations
```

## Quick Start

1. **Set up MATLAB path** (or add directories to path manually):
   ```matlab
   addpath(genpath(pwd));
   ```

2. **Run example simulation**:
   ```matlab
   run_example
   ```

3. **Run single simulation**:
   ```matlab
   params = system_parameters();
   results = main_simulation(params);
   plot_results(results);
   ```

4. **Run gearbox optimization**:
   ```matlab
   params = system_parameters();
   opt_results = optimize_gearbox(params);
   plot_results([], opt_results);
   ```

## Motor Specifications

The simulation uses the **Turnigy Aerodrive SK3-5065-236KV** motor:
- Kv: 236 RPM/V
- Back-EMF constant (K_e): 0.0405 V·s/rad
- Torque constant (K_t): 0.0405 N·m/A
- Winding resistance: 0.019 Ω
- Maximum current: 60 A

## Parameter Configuration

All parameters are defined in `system_parameters.m`. Key parameters include:

- **Physical**: mass, drum radius, initial height
- **Motor**: K_e, K_t, R_winding, J_rotor
- **Losses**: stage efficiencies, friction coefficients
- **Electrical**: load type and parameters
- **Simulation**: time span, solver options, tolerances
- **Optimization**: gearbox ratio range and step size

## Outputs

The simulation provides:
- Time-series data (position, velocity, current, voltage, power)
- Energy flow analysis (gravitational, kinetic, electrical, losses)
- Efficiency metrics (per stage and overall)
- Optimization results (efficiency vs. gearbox ratio)

## Validation

The simulation includes automatic validation checks:
- Energy conservation (error < 1%)
- Unit consistency
- Physical reasonableness
- Numerical stability

## Requirements

- MATLAB R2016b or later
- Optimization Toolbox (optional, for advanced optimization)

## Notes

- All units are SI (kg, m, s, N, N·m, W, J, Ω, V, A)
- The system is kinematically constrained (one dominant rotational degree of freedom)
- Electrical dynamics can be included or treated as quasi-static
- Loss models use a hybrid approach (efficiency + friction)
