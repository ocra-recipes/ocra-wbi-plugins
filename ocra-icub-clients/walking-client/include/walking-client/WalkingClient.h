#ifndef WALKINGCLIENT_H
#define WALKINGCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include <ocra/util/EigenUtilities.h>
#include "walking-client/ZmpPreviewController.h"
#include "walking-client/StepController.h"
#include "walking-client/utils.h"
#include "walking-client/MIQPController.h"
#include <ocra/util/FileOperations.h>
#include <yarp/os/Time.h>
#include "gurobi_c++.h"
#include "Gurobi.h"

class WalkingClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(WalkingClient)

public:
    WalkingClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~WalkingClient ();

   /**
    * Takes all the parameters used by this client from configuration file and parses
    * them through yarp's Resource Finder.
    *
    * Details on group [ZMP_CONTROLLER_PARAMS] in walking-client.ini
    * Options are: 0 - ZMP_CONSTANT_REFERENCE
    *              1 - ZMP_VARYING_REFERENCE
    *              2 - COM_LIN_VEL_CONSTANT_REFERENCE
    * Each of these tests are used to evaluate the correct gains to be used at each
    * level of the control loops. These trajectorie will be used during the tests
    * specified through the option 'test' which takes the values "zmpPreview" or "zmpController".
    * When using this client for the first time on a robot, the gains of the comTask
    * in its corresponding taskSet file must be tuned first as well and later those
    * for the ZmpController class. Therefore we recommend executing this client first
    * as a way of testing the "low" level ComTask control in order to find good kp
    * and kd. Do this by setting 'type' to 2. Data will be saved at the location you
    * specify through the option 'homeDataDir'. After having a good COM velocity
    * tracking at the task level, you want to test the tracking of the zmp
    * controller by setting 'type' to 0. A constant zmp reference is given and the
    * controller gains kfx, kfy, kdx and kdy must be tuned accordingly. Finally, the
    * tracking of varying zmp reference can be tested which takes the zmp from left
    * to right, while the robot stands on both feet.
    */
    bool configure(yarp::os::ResourceFinder &rf);


    /**
     Prints a list of options accepted by the client.
     */
    void printHelp();


    /**
     Reads the raw wrench published for the corresponding analog force/torque sensors in iCub's feet.

     @param whichFoot For which foot is the measurement read.
     @param rawWrench Result of the reading.
     @return True if reading is successful, false otherwise.
     */
    bool readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench);

    yarp::os::BufferedPort<yarp::sig::Vector> portWrenchLeftFoot;

    yarp::os::BufferedPort<yarp::sig::Vector> portWrenchRightFoot;

    /**
     *  Generates a sinusoidal zmp trajectory on the \f$y\f$ expressed in the world reference frame.
     *  This is intended for testing purposes only.
     *
     *  @param tTrans               Time in which you want the ZMP to go from one foot to the other.
     *  @param feetSeparation       Separation between the feet in meters.
     *  @param timeStep             Desired time step.
     *  @param amplitudeFraction    Fraction of the initial feet separation to determine max amplitude of movement.
     *  @param N                    Number of transitions (left to right or right to left).
     *
     *  @return Trajectory of 2D ZMP points.
     */
    std::vector<Eigen::Vector2d> generateZMPTrajectoryTEST(const double tTrans,
                                                           const double feetSeparation,
                                                           const double timeStep,
                                                           const int    amplitudeFraction,
                                                           const int N);

    /**
     *  Returns the current feet separation vector;
     *
     *  @param sep Separation vector.
     *
     *  @return True if all operations proceed well. False otherwise.
     */
    bool getFeetSeparation(Eigen::Vector3d &sep);

    /**
     *  Write the ZMP error (externally computed, thus, any zmp related measurement) to a port.
     *
     *  @param zmpError \f$\mathbf{p} - \mathbf{p_d}\f$
     *
     *  @return True if writing is successful, false otherwise.
     *
     */

    bool publish3dQuantity(yarp::os::BufferedPort<yarp::os::Bottle> &port, Eigen::Vector3d &value);

    /**
     *  Performs a zmp test for the specified type of trajectory.
     *
     *  @param type See ZmpTestType for possible options.
     *  @note To be deprecated as the zmp controller became unnecessary.
     */
    void performZMPTest(ZmpTestType type);


    /**
     Performs a zmpPreviewTest for assessing and tuning of its parameters. This test has been succesfully performed with the iCub platform on Gazebo using the following configuration set in walking-client.ini:

         |  Parameter  | Value |
         | ----------: | :-----|
         |         Nc  | 200   |
         |         nw  | 0.0   |
         |         nb  | 1.0   |
         |         nu  | 1e-6  |

     @param type Trajectory type. \see ZmpTestType.
     */
    void performZMPPreviewTest(ZmpTestType type);

    void performSingleStepTest();

    /**
    * A test where the robot steps forward in a straight line. Only uses ZMP controller.
    */
    void steppingTest();

    /**
     *  Composes a port name with the client name as the suffix. This client name is assumed to be passed through command line options or configuration file as per the policies of yarp's Resource Finder.
     *
     *  @param portName Name of the port without backslashes. e.g. "zmpError:o"
     *  @return A client-name-prepended port name, e.g. "/walkingClient/zmpError:o"
     */
    std::string composePortName(std::string portName);

    void findZMPPreviewControllerParams(yarp::os::ResourceFinder &rf);
    void findGeneralTestsParams(yarp::os::ResourceFinder &rf);
    void findCOMLinVelConstRefParams(yarp::os::ResourceFinder &rf);
    void findZMPConstRefParams(yarp::os::ResourceFinder &rf);
    void findSingleStepTestParams(yarp::os::ResourceFinder &rf);
    void findZMPVaryingReferenceParams(yarp::os::ResourceFinder &rf);
    void findMIQPParams(yarp::os::ResourceFinder &rf);
    void findSteppingTestParams(yarp::os::ResourceFinder &rf);

    /**
     Takes an std::Vector of ZMP trajectories at time \f$k\f$ and outputs the ZMP samples from time \f$k\f$ until \f$k + N_c\f$s, i.e. the ZMP preview window.

     @param fullTraj Vector of horizontal ZMP positions.
     @param from Index in the vector corresponding to the current time instant.
     @param Nc Length of the preview window.
     @param output ZMP samples in the preview window.
     */
    void transformStdVectorToEigenVector(std::vector< Eigen::Vector2d >& fullTraj, int from, int Nc, VectorXd& output);


    /**
     Generates a step-like zmp trajectory for testing/assessment purposes. This assumes a world reference frame with positive x axis pointing forward and positive z axis pointing up.

     @param feetSeparation Separation between the feet.
     @param period Thread period in ms.
     @param duration Duration of the trajectory in seconds.
     @param riseTime Time in seconds at which the step should happen.
     @param constantReferenceY Value of the
     @return Vector of horizontal ZMP positions.
     */
    std::vector< Eigen::Vector2d > generateZMPStepTrajectoryTEST(double feetSeparation, double period, double duration, double riseTime, double constantReferenceY);

    std::vector<Eigen::Vector2d> generateZMPSingleStepTrajectory(double period, double feetSeparation);

    /**
     Generates a straight line step pattern with step targets, step order, and step durations. Used for testing.
    */
    void generateStepPattern();

    /**
     Generates a ZMP trajectory for the STEPPING_TEST. Uses the step pattern generated by @ref generateStepPattern.
    */
    std::vector<Eigen::Vector2d> generateZMPSteppingTrajectory();

    /**
     Manages the step switching for the STEPPING_TEST.
    */
    void startSteppinMotherFucker();

    /**
     Prepares an object of type ocra::TaskState with the com state passed to this method and when doSet is true, applies the control to the robot.

     @param comState 6-dim CoM state vector (horizontal dynamics).
     @param doSet True if the desired state is to be set. False otherwise.
     @todo Change input to this method to just com acceleration.
     */
    void prepareAndsetDesiredCoMTaskState(Eigen::VectorXd comState, bool doSet);



protected:
    /**
     * @todo Dummy initial COM states reference!
     */
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    // Variables for STEPPING_TEST.
    std::vector<FOOT> _stepOrder;
    Eigen::MatrixXd _stepTargets;
    Eigen::VectorXd _stepTargetDurations;
    bool _currentlyStepping;
    bool _waitBeforeNextStep;
    double _waitTimeStart;
    int _currentStepIndex;
    std::vector<Eigen::Vector2d> _steppingTrajectory;
    steppingTestParams _steppingTestParams;


    // General Variables
    std::shared_ptr<ZmpPreviewParams> _zmpPreviewParams;
    std::shared_ptr<ZmpPreviewController> _zmpPreviewController;
    std::shared_ptr<ocra_recipes::TaskConnection> _comTask;
    std::shared_ptr<MIQPController> _miqpController;
    std::shared_ptr<StepController> _stepController;
    std::vector<Eigen::Vector2d> _zmpTrajectory;
    std::vector<Eigen::Vector2d> _singleStepTrajectory;
    ocra::TaskState _desiredComState;
    MIQPParameters _miqpParams;
    Eigen::VectorXd _rawLeftFootWrench;
    Eigen::VectorXd _rawRightFootWrench;
    Eigen::Vector2d _globalZMP;
    Eigen::Vector2d _previousCOM;
    Eigen::Vector2d _previousCOMVel;
    std::string _clientName;
    std::string _robot;
    bool _isTestRun;
    std::string _testType;
    ZmpTestType _zmpTestType;
    std::string _homeDataDir;
    double _comYConstVel;
    double _stopTimeConstComVel;
    double _zmpYConstRef;
    double _stopTimeConstZmp;
    double _riseTimeConstZmp;
    double _trajectoryDurationConstZmp;
    double _constantReferenceY;
    singleStepTestParams _singleStepTestParams;
    double _tTrans;
    int _numberOfTransitions;
    int _amplitudeFraction;
    double _stopTimeVaryingZmp;

    yarp::os::BufferedPort<yarp::os::Bottle> _zmpPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _dcomErrorPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _dComDesPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _dComCurPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _zmpDesPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _zmpCurPort;
    yarp::os::BufferedPort<yarp::os::Bottle> _comCurrent;
    yarp::os::BufferedPort<yarp::os::Bottle> _ddcomCurrent;
    yarp::os::BufferedPort<yarp::os::Bottle> _ddcomFromZMP;
    VectorXd _hkkPrevious;
    bool _firstLoop;
    Eigen::VectorXd zmpRefInPreviewWindow;
    Eigen::VectorXd comVelRefInPreviewWindow;
    Eigen::VectorXd optimalU;
    
    // MIQP-related
    Eigen::VectorXd _X_kn;

};


#endif // TEST_CLIENT_H
