#ifndef STEPPING_DEMO_CLIENT_H
#define STEPPING_DEMO_CLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
// #include <ocra/control/Model.h>

enum COM_SUPPORT_POSITION
{
    LEFT_FOOT_XY,
    RIGHT_FOOT_XY,
    CENTERED_BETWEEN_FEET_XY
};

enum CONTROL_PHASE
{
    MOVE_TO_LEFT_SUPPORT,
    MOVE_TO_RIGHT_SUPPORT,
    MOVE_TO_DOUBLE_SUPPORT
};

enum FOOT_CONTACTS
{
    LEFT_FOOT,
    RIGHT_FOOT
};


class SteppingDemoClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(SteppingDemoClient)

public:
    SteppingDemoClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~SteppingDemoClient ();

protected:
    /**
     This methods does the following:
     1. Define trajectory types (e.g. Min. Jerk)
     2. Define the termination strategy
     3. Instantiates trajectory threads for the left and right foot as well as for the CoM, sets them up (max velocity, error threshold) and finally starts them.
     4. Instantiates ocra::TaskConnection objects for the left and right feet contacts (4 per foot on each corner).
     
     - returns: True if everything has been initialized properly, false otherwise.
     */
    virtual bool initialize();
    
    /**
     *  Stops the trajectory threads of the CoM, left and right feet.
     */
    virtual void release();
    
    /**
     *  Initially waits for two seconds, to make sure that all model states have been correctly updated during the initialization. During the first loop it retrieves left and right feet positions along with the com's.
     */
    virtual void loop();

private:
    ocra_recipes::TaskConnection::Ptr LeftFootContact_BackLeft;
    ocra_recipes::TaskConnection::Ptr LeftFootContact_FrontLeft;
    ocra_recipes::TaskConnection::Ptr LeftFootContact_BackRight;
    ocra_recipes::TaskConnection::Ptr LeftFootContact_FrontRight;

    ocra_recipes::TaskConnection::Ptr RightFootContact_BackLeft;
    ocra_recipes::TaskConnection::Ptr RightFootContact_FrontLeft;
    ocra_recipes::TaskConnection::Ptr RightFootContact_BackRight;
    ocra_recipes::TaskConnection::Ptr RightFootContact_FrontRight;

    /**
     *  Retrieves the 3D position of the "l_sole" frame from the iCub model.
     *
     *  @return 3D position of the left foot.
     */
    Eigen::Vector3d getLeftFootPosition();
    
    /**
     *  Retrieves the 3D position of the "r_sole" frame from the iCub model.
     *
     *  @return 3D position of the right foot.
     */
    Eigen::Vector3d getRightFootPosition();
    
    /**
     *  Retrieves the 3D position of the "CoM" as done by the yarpWholeBodyInterface.
     *
     *  @return 3D position of the COM.
     */
    Eigen::Vector3d getCoMPosition();
    
    void positionCoMOver(COM_SUPPORT_POSITION newSupportPos);

    bool isBalanced();
    
    void deactivateFootContacts(FOOT_CONTACTS foot);
    
    void activateFootContacts(FOOT_CONTACTS foot);

    bool isFootInContact(FOOT_CONTACTS foot);

    void pauseFor(double _pauseDuration);
    
    bool pauseFinished();

    bool liftFoot(FOOT_CONTACTS foot);
    
    bool setDownFoot(FOOT_CONTACTS foot);

    double pauseTriggerTime;
    
    double pauseDuration;
    
    double currentTime;
    
    bool isPausing;

    bool isInLeftSupportMode;
    
    bool isInRightSupportMode;
    
    bool footTrajectoryStarted;

    std::shared_ptr<ocra_recipes::TrajectoryThread> leftFoot_TrajThread;
    std::shared_ptr<ocra_recipes::TrajectoryThread> rightFoot_TrajThread;
    std::shared_ptr<ocra_recipes::TrajectoryThread> com_TrajThread;


    Eigen::Vector3d leftFootHome;
    Eigen::Vector3d rightFootHome;
    Eigen::Vector3d comHome;
    Eigen::Vector3d leftFootTarget;
    Eigen::Vector3d rightFootTarget;

    Eigen::Vector3d newCoMGoalPosition;


    double startTime;
    bool getInitialValues;

    CONTROL_PHASE currentPhase;
    CONTROL_PHASE nextPhase;
    bool isMovingCoM;

};
#endif // STEPPING_DEMO_CLIENT_H
