#ifndef TASKSET1_H
#define TASKSET1_H

#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"


    /* Conversions between types/conventions used in Eigen and WBI */
    class TaskSet1
    {
    public:
        static ISIRCtrlTaskManager getTask(Model& model, orcisir::ISIRController& ctrl);

    };

#endif // TASKSET1_H

