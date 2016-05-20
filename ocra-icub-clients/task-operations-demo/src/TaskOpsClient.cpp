#include "task-operations-demo/TaskOpsClient.h"

TaskOpsClient::TaskOpsClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
, thingToDo(REMOVE_TASK)
{
    // poopoo
}

TaskOpsClient::~TaskOpsClient()
{
    //caca
}

bool TaskOpsClient::initialize()
{

    std::cout << "Begin loop." << std::endl;

    return true;
}

void TaskOpsClient::release()
{
    /* Do nothing. */
}

void TaskOpsClient::loop()
{
    switch (thingToDo) {
        case REMOVE_TASK:
        {
            std::cout << "I am now in phase: REMOVE_TASK." << std::endl;
            removeTask("rightHandCartesian");
            thingToDo = ADD_NEW_TASK;
            yarp::os::Time::delay(2.0);
        }break;

        case ADD_NEW_TASK:
        {
            std::cout << "I am now in phase: ADD_NEW_TASK." << std::endl;
            // addTasks("/home/ryan/tmp_tasks.xml", true);
            thingToDo = ADD_EXISTING_TASK;
            yarp::os::Time::delay(2.0);
        }break;

        case ADD_EXISTING_TASK:
        {
            std::cout << "I am now in phase: ADD_EXISTING_TASK." << std::endl;

            thingToDo = ADD_EXISTING_TASK_NO_OVERWRITE;
            yarp::os::Time::delay(2.0);
        }break;

        case ADD_EXISTING_TASK_NO_OVERWRITE:
        {
            std::cout << "I am now in phase: ADD_EXISTING_TASK_NO_OVERWRITE." << std::endl;

            thingToDo = NOTHING;
            yarp::os::Time::delay(2.0);
        }break;
    }
}
