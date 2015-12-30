#include <testActivity/controlThreadBase.h>
#include <testActivity/trajectoryThread.h>
#include <testActivity/controllerConnection.h>
// #include <yarp/os/Contact.h>
// #include <yarp/os/impl/Dispatcher.h>
// #include <vector>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cout << "No yarp network, quitting\n";
        return 1;
    }

    controllerConnection ctlCon;


    std::vector<std::string> portNames = ctlCon.getTaskPortNames();

    std::cout << "portNames[4]: " << portNames[4] << std::endl;

    TRAJECTORY_TYPE trajType = GAUSSIAN_PROCESS;

    Eigen::MatrixXd waypoints(3,1);
    waypoints <<    0.1,
                    0.1,
                    0.6;

    TERMINATION_STRATEGY termStrategy = WAIT_DEACTIVATE;

    trajectoryThread leftHandTrajThread(10, portNames[4], waypoints, trajType, termStrategy);

    leftHandTrajThread.setGoalErrorThreshold(0.045);

    std::cout << "Thread started." << std::endl;
    leftHandTrajThread.start();

    bool done=false;
    double startTime=yarp::os::Time::now();

    std::cout << "In the while loop..." << std::endl;

    bool p1, p2, p3;
    p1 = true;
    p2 = true;
    p3 = true;
    while(!done)
    {

        if ((yarp::os::Time::now()-startTime)>10.0){
            if(p1){std::cout << "Changing to BACK_AND_FORTH mode:" << std::endl; p1=false;}
            leftHandTrajThread.setTerminationStrategy(BACK_AND_FORTH);
            if ((yarp::os::Time::now()-startTime)>20.0){
                if(p2){std::cout << "Changing to STOP_THREAD mode:" << std::endl; p2=false;}
                leftHandTrajThread.setTerminationStrategy(STOP_THREAD);
                if ((yarp::os::Time::now()-startTime)>30.0){
                    if(p3){std::cout << "Finished while loop!" << std::endl; p3=false;}
                    done=true;
                }
            }
        }
    }

    std::cout << "Stopping thread..." << std::endl;
    leftHandTrajThread.stop();

    std::cout << "Module finished." << std::endl;
    return 0;
}
