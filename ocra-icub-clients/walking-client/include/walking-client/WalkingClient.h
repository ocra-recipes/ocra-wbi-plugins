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
    

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    std::shared_ptr<ZmpControllerParams> _zmpParams;
    std::shared_ptr<ZmpController> _zmpController;

};


#endif // TEST_CLIENT_H
