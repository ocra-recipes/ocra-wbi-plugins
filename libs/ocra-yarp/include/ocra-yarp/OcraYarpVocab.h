#ifndef OCRA_YARP_VOCAB_H
#define OCRA_YARP_VOCAB_H

#include <iostream>
#include "ocra-yarp/OcraYarpTools.h"

namespace ocra_yarp
{

enum OCRA_CONTROLLER_MESSAGE_TAG
{
    // Indicators
    OCRA_FAILURE = 0,
    OCRA_SUCCESS,
    OCRA_WARNING,
    // Task related
    ADD_TASK,
    ADD_TASK_FROM_FILE,
    REMOVE_TASK,
    REMOVE_TASKS,
    GET_TASK_LIST,
    GET_TASK_PORT_LIST,
    // Other
    START_CONTROLLER,
    STOP_CONTROLLER,
    PAUSE_CONTROLLER,
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
