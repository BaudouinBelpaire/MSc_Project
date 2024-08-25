# Waste Collection Pipeline on BlueBoat

## Environment Setup:

![image](https://github.com/user-attachments/assets/2d25fc98-3713-4953-b08d-357243eec0ed)

Install and setup Stonefish simulator: <br />
https://github.com/patrykcieslak/stonefish

Clone the repository to have access to the BlueBoat simulation model on Stonefish: <br />
https://github.com/oceansystemslab/HeriotWattStonefishSim

Replace the launch and scenarios files located in the Simulation folder in the HeriotWattStonefishSim folder previously cloned.

Launch the BlueBoat Simulation: <br />
`roslaunch cola2_stonefish blueboat_simulation.launch`

## Waste Detection:

![image](https://github.com/user-attachments/assets/3041c28c-3ec3-4e12-83a1-97ae093d1ab0)

Install and setup the ROS wrapper for ZED camera: <br />
https://github.com/stereolabs/zed-ros-wrapper

Launch waste detection node:
```
roslaunch waste_collection_pipeline transform.launch
roslaunch waste_collection_pipeline waste_detection.launch
```

## Waste collection: 
![image](https://github.com/user-attachments/assets/72d842c6-bd5d-4ed8-84c1-870c3ea1ccf5)

Launch the service server to compute IK and FK for arm kinematics: <br />
`roslaunch waste_collection_pipeline kinematics.launch`

Launch waste collection script to collect wastes: <br />
`rosrun waste_collection_pipeline main.py`
