Simulation mode - launch simple simulator:
	ros2 launch kia_test sim_load.launch.py auto_sync_mode:=True

With the KIA - launch GNSS node (Applanix) and drive-by-wire (IDAN)
	ros2 launch kia_test kia_load.launch.py

Record a trajectory:
	ros2 launch kia_test record_trajectory.launch.py


Drive the vehicle along the trajectory
	ros2 launch kia_test drive_vehicle.launch.py



