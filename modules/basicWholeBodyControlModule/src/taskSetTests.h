#ifndef TASKSETTESTS_H
#define TASKSETTESTS_H

#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"


    /* Conversions between types/conventions used in Eigen and WBI */
    class TaskSet_initialPosHold
    {
    public:
        static ISIRCtrlTaskManager getTask(Model& model, orcisir::ISIRController& ctrl);

    };
 
    class TaskSet_initialPosZero
    {
    public:
        static ISIRCtrlTaskManager getTask(Model& model, orcisir::ISIRController& ctrl);

    };
    
    
    class TaskSet_initialPosHold_leftHandPos
    {
    public:
        static ISIRCtrlTaskManager getTask(Model& model, orcisir::ISIRController& ctrl);

    };
    
#endif // TASKSETTESTS_H

