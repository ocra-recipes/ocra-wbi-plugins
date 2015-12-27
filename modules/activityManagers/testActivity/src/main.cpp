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
    std::cout << "Thread 1 started." << std::endl;
    myThread.start();

    trajectoryThread myThread2(1000, portNames[2]); //period is 40ms
    std::cout << "Thread 2 started." << std::endl;
    myThread2.start();


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

    std::cout << "Stopping thread 1..." << std::endl;
    myThread.stop();
    std::cout << "Stopping thread 2..." << std::endl;
    myThread2.stop();

    std::cout << "Main finished." << std::endl;
    return 0;
}
