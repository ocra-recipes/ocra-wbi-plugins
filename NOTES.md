#Usage notes for the ISIRWholeBodyController/gOcraController

These notes are written specifically for the iCubParis robots. Please be careful if you are implementing the controller on another platform.

##General

In order to run the controller `wholeBodyDynamicsTree` must be running to do torque estimation. Additionally, as of this writing, the `jointTorqueControl` module must be running for each part. This is generally accomplished by running `codycomoddev --from jtc_[part].ini` for each part.


##iCubParis02

###Setup

Right now the left leg is not working so it has been deactivated in robotInterface. Because of this we have to modify the `.ini` for `wholeBodyDynamicsTree` and `yarpWholeBodyInterface`. These modified files are in:
 - `$CODYCO_SUPERBUILD_ROOT/main/codyco-modules/src/modules/wholeBodyDynamicsTree/app/robots/iCubParis02/wholeBodyDynamicsTree_no_left_leg.ini`
 - `$CODYCO_SUPERBUILD_ROOT/libraries/yarp-wholebodyinterface/app/robots/iCubParis02/yarpWholeBodyInterface_no_left_leg.ini`


 We also have to modify the `.urdf` used by these programs, such that the left leg joints are all `fixed` rather than `revolute`.
  - `$CODYCO_SUPERBUILD_ROOT/libraries/yarp-wholebodyinterface/app/robots/iCubParis02/model_fixed_left_leg.urdf`


 Because the controller uses `yarpWholeBodyInterface` and `jointTorqueControl` we use an additional `yarpWholeBodyInterface` initialization file that ignores the left leg, and uses `JTC`. This is done in:
 - `$CODYCO_SUPERBUILD_ROOT/libraries/yarp-wholebodyinterface/app/robots/iCubParis02/yarpWholeBodyInterface_no_left_leg_jtc.ini`

 where the actuator ports are prepended with /jtc/. Note that this assumes that JTC is running.

 Once this is modified, the [`ISIRWholeBodyController.ini`](/modules/ISIRWholeBodyController/app/robots/iCubParis02/ISIRWholeBodyController.ini) file must too be modified to call `yarpWholeBodyInterface_no_left_leg_jtc.ini`


 Finally we need to calibrate the JTC gains and coeffs in the various `.ini` files located here:
  - `$CODYCO_SUPERBUILD_ROOT/main/codyco-modules/src/devices/jointTorqueControl/app/robots/iCubParis02`



###Launch

  1. Make sure all computers in the cluster are turned on including pc104.
  2. Open `icub-cluster.py` and start yarpserver and yarprun on the computers.
  3. Open `yarpmanager`.
  4. Under `Applications` go to `WBD_JTC_Fixed_Base`. Open the application.
  5. Click `run` (the green play button).
  6. Launch `ISIRWholeBodyController` with whatever task set or sequence desired.


###Lookout for...
 - The function [`getNominalPosture()`](/libs/taskSequences/src/sequenceTools.cpp) tries to access joint indexes on the left leg so when running without the left leg, make sure the appropriate lines are commented out.
 - Some of the Cpp sequences work well in simulation but need much larger gains on the real robot. Additionally, many of them assume you are starting in the home posture and may cause collisions if starting from some other posture.
 - Make sure to recalibrate `wholeBodyDynamicsTree` every 15-30 minutes:
 ```
 yarp rpc /wholeBodyDynamicsTree/rpc:i
 calib all 400
 ```
