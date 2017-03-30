/**
 *  \class StepController
 *
 *  \brief
 *
 *  \note
 *
 *  \author Jorhabib Eljaik
 *
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *
 */

#ifndef _STEPCONTROLLER_H_
#define _STEPCONTROLLER_H_

#include <ocra-recipes/TrajectoryThread.h>
#include "walking-client/utils.h"

class StepController {
public:

    StepController(int periodms, ocra::Model::Ptr model);
    virtual ~StepController();

    /**
     * @todo This max velocity must be compatible with the one specified in the linar constraints
     */
    bool initialize();

    /**
     *  Remember the right and left feet contact tasks? created in initialize().
     * This method asks the ocra-icub-server to deactivate one by one the feet
     * "corner" contact tasks for the specified foot.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     */
    void deactivateFeetContacts(FOOT foot);

    /**
     *  This method asks the ocra-icub-server to activate one by one the feet
     * "corner" contact tasks for the specified foot.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     */
    void activateFeetContacts(FOOT foot);

    
    bool doStepWithMaxVelocity(FOOT foot, Eigen::Vector3d target, double stepHeight);

    /**
     *  Make a step with foot with a specific duration with a specific height.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT.
     *  @param target x,y,z location of where the foot should land.
     *  @param duration How long the step should take.
     *  @param stepHeight How high the foot should lift.
     */
    void step(FOOT foot, Eigen::Vector3d target, double stepDuration, double stepHeight);

    /**
     *  Checks if the step has reached it's target locations and if this is true then re-activates the foot contacts.
     *
     *  @param foot LEFT_FOOT or RIGHT_FOOT (whichever is stepping currently)
     */
    bool isStepFinished(FOOT foot);

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

    void computeMidPoint(FOOT foot, Eigen::Vector3d target, double stepHeight, Eigen::Vector3d & midPoint);
    
    
    void doComputeMidPoint(Eigen::Vector3d currentFootPosition, Eigen::Vector3d target, double stepHeight, Eigen::Vector3d & midPoint);

    bool isTrajectoryFinished(FOOT foot);
    
    /**
     *  Stops the feet trajectory threads. To be called during the release of the
     *  owner thread.
     */
    void stop();
    
    
    /**
     * Returns the 2D coordinates of the contacts defined at the task level.

     @return Matrix of 2D coordinates of the contact points defined at task level. 
     */
    Eigen::MatrixXd getContact2DCoordinates();
    

private:
    std::vector<ocra_recipes::TaskConnection::Ptr> _contactsCollection;
    ocra_recipes::TaskConnection::Ptr _LeftFootContact_BackLeft;
    ocra_recipes::TaskConnection::Ptr _LeftFootContact_FrontLeft;
    ocra_recipes::TaskConnection::Ptr _LeftFootContact_BackRight;
    ocra_recipes::TaskConnection::Ptr _LeftFootContact_FrontRight;
    ocra_recipes::TaskConnection::Ptr _RightFootContact_BackLeft;
    ocra_recipes::TaskConnection::Ptr _RightFootContact_FrontLeft;
    ocra_recipes::TaskConnection::Ptr _RightFootContact_BackRight;
    ocra_recipes::TaskConnection::Ptr _RightFootContact_FrontRight;
    std::shared_ptr<ocra_recipes::TrajectoryThread> _leftFoot_TrajThread;
    std::shared_ptr<ocra_recipes::TrajectoryThread> _rightFoot_TrajThread;
    Eigen::Vector3d target;
    Eigen::Vector3d _leftFootPosition;
    Eigen::Vector3d _rightFootPosition;
    ocra::Model::Ptr _model;
    int _period;

};
#endif
