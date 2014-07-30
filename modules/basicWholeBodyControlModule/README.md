README.md

basicWholeBodyControlModule 


Things to remember:
- The urdf model passed to the controller must match the robot being used or simulated, otherwise the torques will be wrong and potentially cause spastic behavior.
- Make sure that the codyco-superbuild has been compiled with the CMAKE flag: CODYCO_ICUBWBI_USES_EXTERNAL_TORQUE set to ON to use jointTorqueControl
- When building codyco-superbuild make sure USES_EIGEN_320 is set to OFF so that the codyco-isir repo can be built.
- In jointTorqueControl watch out for the GAZEBO_SIM flag. On the real robot the torso DoF indexes are swapped. 
- use ICUB_MAIN_JOINTS to be compatible with jointTorqueControl (not ICUB_MAIN_DYNAMIC_JOINTS).


note: typing in parentheses () are comments
note: <items_here> are to be filled in by you without the <> at the ends



step 1:

	TERM A:
	$ yarpserver (add "--read" option to read local config file)


step 2:

	TERM B:
	$ gazebo
	
	click the "insert" tab and drag and drop the "icub_fixed" model into the scene

step 3 (optional):
	
	TERM C:
	$ robotMotorGui
	
	enter the name "icubGazeboSim" and click the button
	(this is just to make sure the proper control modes are activated)
	
	
	


==== To run directly in Gazebo ====

step 4:

	TERM D:
	$ cd <path/to/codyco-isir>/basicWholeBodyControlModule/build/
	$ ./basicWholeBodyControlModule --robot icubGazeboSim --urdf ../src/icub_simulation.urdf --headV2








==== To run using Joint Torque Control ====
(This should be used to simulate the interface with the real robot's local torque controllers. Be wary of the configuration files and the gains on the low level controllers.)

step 4:

	TERM D:
	$ cd <path/to/codyco-superbuild>/main/codyco-modules/src/modules/jointTorqueControl/app/conf/
	$ jointTorqueControl --from icubGazeboSim_Conf_file_for_JTC_and_wholeBody_control.ini (or whatever .ini file you need)


step 5:

	TERM E:
	$ cd <path/to/codyco-isir>/basicWholeBodyControlModule/build/
	$ ./basicWholeBodyControlModule --robot icubGazeboSim --urdf ../src/icub_simulation.urdf --headV2 --uses_external_torque_control

