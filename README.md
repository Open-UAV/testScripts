# Scripts to control drone in PX4 Gazebo environment

Qualifying: 
Run `docking_demo.py` to code to attach and detach an object with a PX4 drone.
This script will also set mode to Offboard and arm the vehicle.

This script publishes to attach and detach commands, which require the following repository code to be setup and running.
https://github.com/Open-UAV/gazebo_ros_link_attacher 



Run `position_control_demo.py` to move the drone in a diamond pattern.
This script will need you manually set the mode to offboard and arm the vehicle.
Commands to set mode and arm the vehicle.
```
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /mavros/cmd/arming "value: true"
```
 	
  
Phase II:
 Run `multi_vehicle_position_control_demo.py.py` to move the drone along a sequence of position setpoints charting the Phase II mission goals. The script ends with tracking the rover position at a safe distance. 