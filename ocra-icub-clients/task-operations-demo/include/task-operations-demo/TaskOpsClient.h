#ifndef EXAMPLE_CLIENT_H
#define EXAMPLE_CLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>

enum THINGS_TO_DO
{
    REMOVE_TASK,
    ADD_NEW_TASK,
    ADD_EXISTING_TASK,
    ADD_EXISTING_TASK_NO_OVERWRITE,
    NOTHING
};


class TaskOpsClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(TaskOpsClient)

public:
    TaskOpsClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~TaskOpsClient ();

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    THINGS_TO_DO thingToDo;
};


#endif // EXAMPLE_CLIENT_H
