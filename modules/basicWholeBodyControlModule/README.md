README.md

basicWholeBodyControlModule 


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

