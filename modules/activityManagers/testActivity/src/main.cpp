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

    std::cout << "portNames[1]: " << portNames[1] << std::endl;
    std::cout << "portNames[2]: " << portNames[2] << std::endl;

    trajectoryThread myThread(100, portNames[1]); //period is 40ms
    myThread.start();

    trajectoryThread myThread2(1000, portNames[2]); //period is 40ms
    myThread2.start();


    bool done=false;
    double startTime=yarp::os::Time::now();
    while(!done)
    {
        if ((yarp::os::Time::now()-startTime)>2.0)
            done=true;
    }

    myThread.stop();
    myThread2.stop();

    return 0;
}
