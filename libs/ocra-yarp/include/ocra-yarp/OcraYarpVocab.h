#ifndef OCRA_YARP_VOCAB_H
#define OCRA_YARP_VOCAB_H

#include <iostream>
#include "ocra-yarp/OcraYarpTools.h"

namespace ocra_yarp
{

enum OCRA_CONTROLLER_MESSAGE
{
    // General indicators
    OCRA_FAILURE = 0,
    OCRA_SUCCESS,
    OCRA_WARNING,

    // Controller requests
    GET_CONTROLLER_STATUS,
    GET_WBI_CONFIG_FILE_PATH,
    GET_ROBOT_NAME,
    GET_IS_FLOATING_BASE,

    START_CONTROLLER,
    STOP_CONTROLLER,
    PAUSE_CONTROLLER,

    // Controller status indicators
    CONTROLLER_RUNNING,
    CONTROLLER_STOPPED,
    CONTROLLER_PAUSED,

    // Task requests
    ADD_TASK,
    ADD_TASK_FROM_FILE,
    REMOVE_TASK,
    REMOVE_TASKS,
    REMOVE_TASK_PORT,
    GET_TASK_LIST,
    GET_TASK_PORT_LIST,
    // Other
    HELP
};

enum OCRA_CONTROLLER_TYPE
{
    WOCRA_CONTROLLER = 10,
    GOCRA_CONTROLLER = 11,
    HOCRA_CONTROLLER = 12
};

} // namespace ocra_yarp
#endif // OCRA_YARP_VOCAB_H
