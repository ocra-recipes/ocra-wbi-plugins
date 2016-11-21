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

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    ocra_recipes::TrajectoryThread::Ptr rootTrajThread;
};


#endif // TEST_CLIENT_H
