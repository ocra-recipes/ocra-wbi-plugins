#include "example-client/ExampleClient.h"

ExampleClient::ExampleClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    // poopoo
}

ExampleClient::~ExampleClient()
{
    //caca
}

bool ExampleClient::initialize()
{
    startTime = yarp::os::Time::now();
    trigger = true;

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;

    waypoints.resize(3,3);
    waypoints <<     0.20, 0.25, 0.15,
                    -0.06, 0.10, 0.30,
                     0.55, 0.85, 0.65;

    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::BACK_AND_FORTH;

    leftHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "leftHandCartesian", waypoints, trajType, termStrategy);

    // leftHandTrajThread->setDisplacement(0.2);
    leftHandTrajThread->setGoalErrorThreshold(0.03);
    leftHandTrajThread->setMaxVelocity(0.2);


    done=false;

    p1 = true;
    p2 = true;
    p3 = true;

    waitTime = 1.0;

    std::cout << "Begin loop." << std::endl;

    return true;
}

void ExampleClient::release()
{
    /* Do nothing. */
}

void ExampleClient::loop()
{

    while(!done && ((yarp::os::Time::now() - startTime) > waitTime))
    {
        if(trigger){
            leftHandTrajThread->start();
            std::cout << "Traj thread started." << std::endl;
            trigger = false;
        }

        if ((yarp::os::Time::now()-startTime)>15.0){
            if(p1){
                p1=false;
                std::cout << "Changing to CYCLE mode." << std::endl;
                leftHandTrajThread->setTerminationStrategy(ocra_recipes::CYCLE);
            }
            if ((yarp::os::Time::now()-startTime)>30.0){
                if(p2){
                    p2=false;
                    std::cout << "Pausing trajectory." << std::endl;
                    leftHandTrajThread->pause();
                }
                if ((yarp::os::Time::now()-startTime)>40.0){
                    if(p3){
                        p3=false;
                        std::cout << "Un-pausing trajectory." << std::endl;
                        leftHandTrajThread->unpause();
                        done=true;
                    }
                }
            }
        }
    }


}
