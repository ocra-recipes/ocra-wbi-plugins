#ifndef _UTILS_H_
#define  _UTILS_H_
#include <string>

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
    COM_LIN_VEL_CONSTANT_REFERENCE, /*Constant COM linear velocity. The zmp controller does nothing. Used to tuned the com task level gains.*/
    SINGLE_STEP, /* Performs one single step with the right foot using also the zmp preview controller */
    STEPPING_TEST /* Performs one single step with the right foot using also the zmp preview controller */
};

struct singleStepTestParams {
    double totalDuration;
    double offset;
    double SSduration;
    double riseTime;
    double stepLength;
    double stepHeight;
};

struct steppingTestParams {
    int    nSteps;
    double stepDuration;
    double stepLength;
    double stepHeight;
};

struct MIQPParameters {
    /* Size of preview window */
    unsigned int N;
    /* Constant com height */
    double cz;
    /* Gavity acceleration */
    double g;
    /* Thread period */
    unsigned int dt;
    /* Home directory where logged data will be saved */
    std::string home;
    /* Walking Performance Cost Weight */
    double ww;
    /* Balance Performance Cost Weight */
    double wb;
    /* Jerk regularization Cost Weight */
    double wu;
    /* Regularization weight to avoid resting on one foot */
    double wss;
    /* Regularization weight to minimize stepping */
    double wstep;
    /* Regularization weight on delta*/
    double wdelta;
    /* X Boundary in Constancy constraints */
    double sx_constancy;
    /* Y Boundary in Constancy constraints */
    double sy_constancy;
    /* X Boundary in Single Support Constraints */
    double sx_ss;
    /* Y Boundary in Single Support Constraints */
    double sy_ss;
    /* CoM State Reference Selection Vector. If your CoM State Reference consists only of forward velocity
     * then this vector is: [0 0 1 1 0 0] */
    double hx_ref;
    double hy_ref;
    double dhx_ref;
    double dhy_ref;
    double ddhx_ref;
    double ddhy_ref;
    /* Add Shape Constraints? */
    bool shapeConstraints;
    /* Add Admissibility Constraints? */
    bool admissibilityConstraints;
    /* Add CoP Constraints? */
    bool copConstraints;
    /* Add walking constraints? */
    bool walkingConstraints;
    /* Add regularization terms?*/
    bool addRegularization;
    /* robot name */
    std::string robot;
};

#define STATE_VECTOR_SIZE 16
#define INPUT_VECTOR_SIZE 12

#endif
