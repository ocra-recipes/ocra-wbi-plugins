#ifndef TRAJECTORYTHREAD_H
#define TRAJECTORYTHREAD_H value


#include <testActivity/controlThreadBase.h>
#include <iostream>

class trajectoryThread : public controlThreadBase
{

public:
    trajectoryThread(int period, const std::string& taskPortName, const std::string& trajectoryType = "minimumJerk");

    // virtual bool threadInit();
    // virtual void threadRelease();
    // virtual void run();
    virtual bool ct_threadInit();
    virtual void ct_threadRelease();
    virtual void ct_run();

    virtual std::string getThreadType(){return "trajectoryThread";}


private:
    std::string trajType;

};
#endif
