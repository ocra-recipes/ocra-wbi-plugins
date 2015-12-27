#include <testActivity/trajectoryThread.h>

trajectoryThread::trajectoryThread(int period, const std::string& taskPortName, const std::string& trajectoryType):
controlThreadBase(period, taskPortName),
trajType(trajectoryType)
{
    setThreadType("trajectoryThread");
}

bool trajectoryThread::ct_threadInit()
{
    return true;
}

void trajectoryThread::ct_threadRelease()
{
    std::cout<< "trajectoryThread: Trajectory thread finished.\n";
}

void trajectoryThread::ct_run()
{


}
