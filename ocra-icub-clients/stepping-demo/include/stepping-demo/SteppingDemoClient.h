#ifndef STEPPING_DEMO_CLIENT_H
#define STEPPING_DEMO_CLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
// #include <ocra/control/Model.h>
#include <ocra/util/ErrorsHelper.h>

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
     The initialization consists of the following:
     1. Defines trajectory types (e.g. Min. Jerk)
     2. Defines the termination strategy
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
    
    /**
     *  Sets COM trajectory waypoints according to the desired support position:
     *  LEFT_FOOT_XY: The COM goal position is set to be on top of the left foot.
     *  RIGHT_FOOT_XY: The COM goal position is set to be on top of the right foot.
     *  CENTERED_BETWEEN_FEET_XY: The COM goal position is set between both feet.
     *  The height of the com is always the current COM's height. 
     *  Since the thread must have started before calling this method, it will update the current waypoint trajectory it has or start it.
     *
     *  @param newSupportPos New support position.
     */
    void positionCoMOver(COM_SUPPORT_POSITION newSupportPos);

    /**
     *  Uses the norm of the COM and joint velocities to understand if the robot is balanced.
     *
     *  @return True when both the norm of the COM and joint velocities are below comVelThreshold and jointThreshold.
     *  @todo A more balance-centered criterion should be used for this, ZMP-based for example.
     */
    bool isBalanced();
    
    /**
     *  Remember the right and left feet contact tasks? created in initialize(). This method acts the ocra-icub-server to deactivate one by one the feet "corner" contact tasks for the specified foot.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     */
    void deactivateFootContacts(FOOT_CONTACTS foot);
    
    /**
     *  This method asks the ocra-icub-server to activate one by one the feet "corner" contact tasks for the specified foot.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     */
    void activateFootContacts(FOOT_CONTACTS foot);

    /**
     *  If the foot height is less than or equal to footContactReleaseThreshold, the feet contacts tasks are activated.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     *
     *  @return True if the activation of the contact happens successfully. False if the foot's elevation is not under the footContactReleaseThreshold.
     */
    bool isFootInContact(FOOT_CONTACTS foot);

    /**
     *  Mainly sets isPausing to true and stores the time at which the pause was called.
     *
     *  @param _pauseDuration Pause duration.
     */
    void pauseFor(double _pauseDuration);
    
    /**
     *  Sets isPausing to false when pauseDuration has passed.
     *
     *  @return True when the pause duration is over. False otherwise.
     */
    bool pauseFinished();

    /**
     *  Sets the right or left foot waypoint to rightFootTarget or leftFootTarget accordingly (where the foot will be lifted). These values are harcoded in the loop() method and updated only during the very first iteration when the variable getInitialValues is set to true.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT
     *
     *  @return True after the foot contact is deactivated.
     */
    bool liftFoot(FOOT_CONTACTS foot);
    
     /**
     *  Sets the right or left foot waypoint to rightFootTarget or leftFootTarget accordingly (where the foot will be lifted). 
     *  These values are harcoded in the loop() method and updated only during the very first iteration when the 
     *  variable getInitialValues is set to true. 
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT
     *  @param isLeftFootInContact  Updates the current state of the left foot contact.
     *  @param isRightFootInContact Updates the current state of the right foot contact.
     *
     *  @return True after the foot contact is deactivated.
     */
    bool liftFoot(FOOT_CONTACTS foot, bool isLeftFootInContact, bool isRightFootInContact);
    
    /**
     *  Sets the right or left foot waypoint to rightFootHome or leftFootHome accordingly (where the foot will land). These values are harcoded in the loop() method and updated only during the very first iteration when the variable getInitialValues is set to true.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT
     *
     *  @return True only after the foot has established contact again and the trajectory is over. False while the foot is still moving back down looking for contact or simply, the goal hasn't been attained.
     */
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
