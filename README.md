# Fuzzy Control of a Wheeled Mobile Robot

## MATLAB Robot Control Simulation

### Overview

This MATLAB script simulates the control of a robot navigating through predefined paths under various conditions.

### Features

- Simulates robot motion along circular and square paths.
- Handles different scenarios, including parameter changes, measurement noise, input saturation, and external disturbances.
- Implements fuzzy and supervisor control strategies for trajectory tracking.
- Includes functions for calculating reference paths, fuzzy control, supervisor control, and total control.

### Usage

1. Run the script `robot_control_simulation.m` in MATLAB.
2. Select the desired path type and problem type to simulate.
3. Observe the robot's trajectory and control behavior in the generated plots.

### Contents

- **main.m**: Main script for simulating robot control.
- **ref_path.m**: Function to calculate reference paths for trajectory tracking.
- **fuzzy_controller.m**: Function implementing fuzzy control for trajectory tracking.
- **supervisor_control.m**: Function implementing supervisor control for trajectory tracking.
- **total_control.m**: Function to calculate total control input based on fuzzy and supervisor control outputs.
- **saturation.m**: Function to limit the input within a specified range to prevent control signal saturation.

### License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

**Parisa Mohammadi**

