# Fuzzy Control of Wheeled a Mobile Robot
**MATLAB Robot Control Simulation**  
**Overview:**   
This MATLAB script simulates the control of a robot navigating through predefined paths under various conditions.  

**Features:**  
-Simulates robot motion along circular and square paths.  
-Handles different scenarios, including parameter changes, measurement noise, input saturation, and external disturbances.  
-Implements fuzzy and supervisor control strategies for trajectory tracking.   
-Includes functions for calculating reference paths, fuzzy control, supervisor control, and total control.   
**Usage:**  
-Run the script robot_control_simulation.m in MATLAB.     
-Select the desired path type and problem type to simulate.     
-Observe the robot's trajectory and control behavior in the generated plots.    

**Contents:**
-main.m: Main script for simulating robot control.    
-ref_path.m: Function to calculate reference paths for trajectory tracking.    
-fuzzy_controller.m: Function implementing fuzzy control for trajectory tracking.    
-supervisor_control.m: Function implementing supervisor control for trajectory tracking.  
-total_control.m: Function to calculate total control input based on fuzzy and supervisor control outputs.
