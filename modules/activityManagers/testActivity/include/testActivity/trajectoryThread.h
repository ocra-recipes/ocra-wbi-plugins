#ifndef TRAJECTORYTHREAD_H
#define TRAJECTORYTHREAD_H value


#include <testActivity/controlThreadBase.h>
#include <iostream>

class trajectoryThread : public controlThreadBase
{

public:
    trajectoryThread(int period, const std::string& taskPortName, const std::string& trajectoryType = "minimumJerk");

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:
    std::string trajType;

};
#endif
