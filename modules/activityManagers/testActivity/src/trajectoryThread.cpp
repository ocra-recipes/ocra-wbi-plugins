#include <testActivity/trajectoryThread.h>

trajectoryThread::trajectoryThread(int period, const std::string& taskPortName, const std::string& trajectoryType):
controlThreadBase(period, taskPortName),
trajType(trajectoryType)
{
    controlThreadType = "trajectoryThread";
}

bool trajectoryThread::threadInit()
{
    return true;
}

void trajectoryThread::threadRelease()
{
    std::cout<< "trajectoryThread:stopping the robot\n";
    std::cout<< "Done, goodbye from trajectoryThread\n";
}

void trajectoryThread::run()
{
    yarp::os::Bottle b;
    b.addString(getThreadType());
    outputPort.write(b);
}
