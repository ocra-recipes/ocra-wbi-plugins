#ifndef WALKINGCLIENT_H
#define WALKINGCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
#include "walking-client/ZmpPreviewController.h"
#include "walking-client/ZmpController.h"

class WalkingClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(WalkingClient)

public:
    WalkingClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~WalkingClient ();
    
    bool readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench);
    
    yarp::os::BufferedPort<yarp::sig::Vector> portWrenchLeftFoot;
    
    yarp::os::BufferedPort<yarp::sig::Vector> portWrenchRightFoot;
    
    /**
     *  Generates a sinusoidal zmp trajectory on the \f$y\f$ expressed in the world reference frame.
     *  This is intended for testing purposes only.
     *
     *  @param tTrans         Time in which you want the ZMP to go from one foot to the other.
     *  @param feetSeparation Separation between the feet in meters.
     *  @param timeStep       Desired time step.
     *  @param N              Number of transitions (left to right or right to left).
     *
     *  @return Trajectory of 2D ZMP points.
     */
    std::vector<Eigen::Vector2d> generateZMPTrajectoryTEST(const double tTrans,
                                                           const double feetSeparation,
                                                           const double timeStep,
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
    bool publishZMPError(Eigen::Vector2d &zmpError);
    

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    std::shared_ptr<ZmpControllerParams> _zmpParams;
    std::shared_ptr<ZmpController> _zmpController;
    std::shared_ptr<ocra_recipes::TaskConnection> _comTask;
    std::vector<Eigen::Vector2d> _zmpTrajectory;
    ocra::TaskState _desiredComState;
    bool _isTestRun;
    
    yarp::os::BufferedPort<yarp::os::Bottle> _zmpPort;

};


#endif // TEST_CLIENT_H
