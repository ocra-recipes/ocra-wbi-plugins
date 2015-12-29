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

    bool stopAtGoal = true;
    bool backAndForth = true;

    trajectoryThread leftHandTrajThread(10, portNames[4], waypoints, trajType, stopAtGoal, backAndForth);

    std::cout << "Thread started." << std::endl;
    leftHandTrajThread.start();

    bool done=false;
    double startTime=yarp::os::Time::now();

    std::cout << "In the while loop..." << std::endl;

    while(!done)
    {

        if ((yarp::os::Time::now()-startTime)>30.0){
            std::cout << "Finished while loop!" << std::endl;
            done=true;
        }
    }

    std::cout << "Stopping thread..." << std::endl;
    leftHandTrajThread.stop();

    std::cout << "Module finished." << std::endl;
    return 0;
}
