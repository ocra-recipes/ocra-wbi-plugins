#ifndef WALKINGCLIENT_H
#define WALKINGCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include <ocra/util/EigenUtilities.h>
#include "walking-client/ZmpPreviewController.h"
#include "walking-client/ZmpController.h"
#include <ocra/util/FileOperations.h>
#include <yarp/os/Time.h>

/**
 Performs tests helpful to tune the gains used by the zmp and zmp preview controllers as well as the COM task. See configure() for additional information.
 */
enum ZmpTestType {
    ZMP_CONSTANT_REFERENCE=0, /*Constant zmp reference (step setpoint). Can be specified through config file with value 0.*/
    ZMP_VARYING_REFERENCE, /*Sinusoidal zmp reference. The parameters can be specified through config file with value 1.*/
    COM_LIN_VEL_CONSTANT_REFERENCE /*Constant COM linear velocity. The zmp controller does nothing. Used to tuned the com task level gains.*/
};

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
    
    void printHelp();

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
     *  Performs one of three possible ZMP tests:
     *
     *  @param type See ZmpTestType for possible options.
     */
    void performZMPTest(ZmpTestType type);
    
    void performZMPPreviewTest(ZmpTestType type);

    /**
     *  Composes a port name with the client name as the suffix. This client name is assumed to be passed through command line options or configuration file as per the policies of yarp's Resource Finder.
     *
     *  @param portName Name of the port without backslashes. e.g. "zmpError:o"
     *
     *  @return A client-name-prepended port name, e.g. "/walkingClient/zmpError:o"
     */
    std::string composePortName(std::string portName);

    void transformStdVectorToEigenVector(std::vector< Eigen::Vector2d >& fullTraj, int from, int Nc, VectorXd& output);
    
    std::vector< Eigen::Vector2d > generateZMPStepTrajectoryTEST(double feetSeparation, double period, double duration, double riseTime, double constantReferenceY);
    
protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    std::shared_ptr<ZmpControllerParams> _zmpParams;
    std::shared_ptr<ZmpController> _zmpController;
    std::shared_ptr<ZmpPreviewParams> _zmpPreviewParams;
    std::shared_ptr<ZmpPreviewController> _zmpPreviewController;
    std::shared_ptr<ocra_recipes::TaskConnection> _comTask;
    std::vector<Eigen::Vector2d> _zmpTrajectory;
    ocra::TaskState _desiredComState;
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

};


#endif // TEST_CLIENT_H
