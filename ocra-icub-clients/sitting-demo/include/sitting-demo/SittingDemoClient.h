#ifndef SITTINGDEMOCLIENT_H
#define SITTINGDEMOCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>


class SittingDemoClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(SittingDemoClient)

public:
    SittingDemoClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~SittingDemoClient ();
    virtual bool configure(yarp::os::ResourceFinder &rf);


protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:

    void moveCom();
    std::string taskName;
    ocra_recipes::TaskConnection::Ptr comTask;
    ocra_recipes::TrajectoryThread::Ptr comTrajThread;
    double xDisp, yDisp, zDisp;
    Eigen::Vector3d currentDesiredPosition;
    Eigen::MatrixXd Kp, Kd;
};


#endif // TEST_CLIENT_H
