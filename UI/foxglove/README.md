# Foxglove Utility Tools for Draco3
*Data collection and operation system developed by Gabriel Moore (2023-2024)*

## Usage
### Teleoperations and Simulation Functionality
***note**: currently a sim-only tool requiring pybullet simulation to run*\
\
**Using Foxglove**: UI/foxglove/UI_launcher.py --visualize=foxglove\
**Using Meshcat (legacy)**: UI/foxglove/UI_launcher.py --visualize=meshcat

### Displaying Robot in Foxglove
Launch the simulator (e.g., python simulator/pyblullet/draco_main.py) 
then run the UI launcher with the `--visualize=foxglove` flag. The first
time it launches, you might need to specify the path to the URDF online:
https://raw.githubusercontent.com/shbang91/rpc/feature/foxglove-ui/robot_model/draco/draco_fox_web.urdf
and set the `Scene/Mesh up axis` to `Z-up`.