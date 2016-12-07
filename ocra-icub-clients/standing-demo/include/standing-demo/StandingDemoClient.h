#ifndef SITTINGDEMOCLIENT_H
#define SITTINGDEMOCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>


class StandingDemoClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(StandingDemoClient)

public:
    StandingDemoClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~StandingDemoClient ();
    bool configure(yarp::os::ResourceFinder &rf);

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    bool useMinJerk;
    ocra_recipes::TrajectoryThread::Ptr comTrajThread;
    ocra_recipes::TrajectoryThread::Ptr rootTrajThread;
    // ocra_recipes::TaskConnection::Ptr comTask;
    // ocra_recipes::TaskConnection::Ptr rootTask;
    ocra_recipes::TaskConnection::Ptr leftLegContactTask;
    ocra_recipes::TaskConnection::Ptr rightLegContactTask;

    double startTime;
    double contactReleaseDelay;
    bool contactsReleased;

    void deactivateLegContacts();

};


#endif // TEST_CLIENT_H
