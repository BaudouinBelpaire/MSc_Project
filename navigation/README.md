# Navigation:

## PID Controller:
Run PID controller for navigation on Stonefish simulation:
'rosrun navigation PID.py'

## Hydrodynamics Parameters:

Collect thrust and displacements data on Simulation:
'rosrun navigation parameters_estimation.py'

Compute hydrodynamics parameters based on collected data:
'python3 parameters_calculation.py'

