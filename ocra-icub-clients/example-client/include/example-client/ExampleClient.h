#ifndef EXAMPLE_CLIENT_H
#define EXAMPLE_CLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>
// #include <ocra/control/Model.h>

class ExampleClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(ExampleClient)

public:
    ExampleClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~ExampleClient ();

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:

    double startTime;
    double waitTime;
    bool trigger;
    bool done;
    Eigen::MatrixXd waypoints;

    std::shared_ptr<ocra_recipes::TrajectoryThread> leftHandTrajThread;

    bool p1, p2, p3;
};


#endif // EXAMPLE_CLIENT_H
