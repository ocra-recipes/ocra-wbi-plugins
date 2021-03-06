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

    ocra_recipes::TaskConnection leftHandTask("LeftHandCartesian");
    Eigen::Vector3d initialPos = leftHandTask.getTaskState().getPosition().getTranslation();

    // std::cout << "\n\n\n" << std::endl;
    // std::cout << "task pose: " << leftHandTask.getTaskState().getPosition() << std::endl;
    // std::cout << "hand pose: " << model->getSegmentPosition("l_hand") << std::endl;
    // std::cout << "\n\n\n" << std::endl;

    waypoints.resize(3,4);
    waypoints.col(0) = initialPos + Eigen::Vector3d(0.0,  0.0,  0.2);
    waypoints.col(1) = initialPos + Eigen::Vector3d(0.0, -0.2,  0.2);
    waypoints.col(2) = initialPos + Eigen::Vector3d(0.0, -0.2,  0.0);
    waypoints.col(3) = initialPos + Eigen::Vector3d(0.0,  0.2,  0.0);


    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::BACK_AND_FORTH;

    leftHandTrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(10, "LeftHandCartesian", waypoints, trajType, termStrategy);

    // leftHandTrajThread->setDisplacement(0.2);
    leftHandTrajThread->setGoalErrorThreshold(0.07);
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
