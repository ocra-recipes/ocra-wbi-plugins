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

    trajectoryThread leftHandTrajThread(10, portNames[4]); //period is 10ms
    std::cout << "Thread started." << std::endl;
    leftHandTrajThread.start();

    bool done=false;
    double startTime=yarp::os::Time::now();

    std::cout << "In the while loop..." << std::endl;

    while(!done)
    {

        if ((yarp::os::Time::now()-startTime)>10.0){
            std::cout << "Finished while loop!" << std::endl;
            done=true;
        }
    }

    std::cout << "Stopping thread..." << std::endl;
    leftHandTrajThread.stop();

    std::cout << "Module finished." << std::endl;
    return 0;
}
