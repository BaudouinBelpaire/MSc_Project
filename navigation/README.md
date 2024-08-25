# Navigation:

## PID Controller:
![image](https://github.com/user-attachments/assets/0cd80bb8-eec0-4783-9e57-568983761b2c)

Run PID controller for navigation on Stonefish simulation: <br />
`rosrun navigation PID.py`

## Hydrodynamics Parameters:

Collect thrust and displacements data on Simulation: <br />
`rosrun navigation parameters_estimation.py`

Compute hydrodynamics parameters based on collected data: <br />
`python3 parameters_calculation.py`

## Path Planner for MPC:
![image](https://github.com/user-attachments/assets/6da8602d-943f-416a-a0ea-33ad4e181d3e)

Run the 'MPC_path_planner.m' file on MATLAB to get the acceleration, velocity, and position profile based on specific maximum velocity and acceleration
