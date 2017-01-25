#ifndef _UTILS_H_
#define  _UTILS_H_

enum FOOT {
    LEFT_FOOT,
    RIGHT_FOOT
};

/**
 Performs tests helpful to tune the gains used by the zmp and zmp preview controllers as well as the COM task. See configure() for additional information.
 */
enum ZmpTestType {
    ZMP_CONSTANT_REFERENCE=0, /*Constant zmp reference (step setpoint). Can be specified through config file with value 0.*/
    ZMP_VARYING_REFERENCE, /*Sinusoidal zmp reference. The parameters can be specified through config file with value 1.*/
    COM_LIN_VEL_CONSTANT_REFERENCE /*Constant COM linear velocity. The zmp controller does nothing. Used to tuned the com task level gains.*/
};


#endif
