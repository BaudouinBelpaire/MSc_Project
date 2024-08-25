# Navigation:

## PID Controller:
Run PID controller for navigation on Stonefish simulation: <br />
`rosrun navigation PID.py`

## Hydrodynamics Parameters:

Collect thrust and displacements data on Simulation: <br />
`rosrun navigation parameters_estimation.py`

Compute hydrodynamics parameters based on collected data: <br />
`python3 parameters_calculation.py`

