Bearing-Only Target Localisation and Circumnavigation (BoTLC)

Description:  
This MATLAB script implements and compares two models for bearing-only target localisation and circumnavigation:

- A baseline predefined-time (Sui 2025) model  
- A proposed intermittent-measurement (IM-PDT) model  

The script supports both simulated intermittent measurements and experimental dataset input.

=========== File Structure ===========

- Datasets          (stored experimental datasets)  
- sig.m             (Helper function)  
- linespecer.m      (Helper function)  
- PDT_IM.m          (Proposed IM-PDT algorithm function)  
- Sui2025.m         (Sui's PDT algorithm function)  
- Main.m            (Main function file) <------------------- Run from this file  

=========== Main.m Structure ===========

Section A: Initialise simulation variables  
Section B: Generate intermittent measurement (IM) mask  
Section C: Read experimental dataset                 (optional: uncomment section to read in dataset)  
Section D: Run BoTLC algorithms  
Section E: Separate available/unavailable measurement data  
Section F: Plot results                              (optional: uncomment experimental plotting if required)  
Section G: Convergence verification (incomplete, only works for simulations only)  
Section H: Helper functions  

=========== How to Run (Default - Simulation Mode) ===========

Open up the Main.m file  
Run the script directly in MATLAB  

The script will:  
- Generate a random intermittent measurement mask  
- Run both models  
- Plot trajectory and error results  

No changes are required for this mode.

=========== How to Run (Default - Experimentation Mode) ===========

IMPORTANT: Using Experimental Data  

If you want to use a dataset instead of simulated data, you MUST do BOTH of the following:

Uncomment ALL of Section C  

This section:  
- Loads the dataset  
- Extracts initial conditions  
- Computes time and timestep values  
- Reads the intermittent measurement mask from the dataset  

Uncomment the experimental plotting lines in Section F  

These include:  
- Agent trajectory plot (experiment path)  
- Estimation error plot (experiment data)  
- Tracking error plot (experiment data)  

If you do not do BOTH steps, the script may fail or produce incorrect results.

====================================================================

Key Parameters:

init_x = [0; 0];              : Target's true position  
init_x_hat = [-1.25; 1.8];    : Target's initial estimated position  
init_y = [-0.92; 2];          : Agent's initial position  
T                             : Total simulation time  
tau                           : Time step  
d_des                         : Desired distance from target  

a1, a2                        : Predefined-time exponents (0 < a < 1)  
Tc1                           : Estimator convergence time  
Tc2                           : Controller convergence time (Tc2 >= Tc1)  
kw                            : Tangential gain  

Outputs:

- Agent Trajectory  
- Estimation Error  
- Tracking Error  

Intermittent Measurement (IM):

The IM signal is a boolean mask:  
1 = measurement available  
0 = measurement unavailable  

Two modes:

- Simulated (Section B)  
- Dataset-driven (Section C)  

Notes:

- Section G (verification) is incomplete and may require fixing  
- Dataset format must match expected column structure  
- Time handling differs between simulation and experimental data  
- Ensure dataset file path is correct before running Section C  
