#ifndef TASKSETTESTS_H
#define TASKSETTESTS_H

#include "ISIRCtrlTaskManager.h"
// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"

#include "orcWbiModel.h"


    /* Conversions between types/conventions used in Eigen and WBI */
    class TaskSet_initialPosHold
    {
    public:
        static ISIRCtrlTaskManager getTask(Model& model, orcisir::ISIRController& ctrl);

    };
 
    class TaskSet_initialPosZero
    {
    public:
        static ISIRCtrlTaskManager getTask(orcWbiModel& model, orcisir::ISIRController& ctrl);

    };

    class TaskSet_initialPosHold_CoMPos_BothHandPos
    {
    public:
        static ISIRCtrlTaskManager getTask(orcWbiModel& model, orcisir::ISIRController& ctrl);

    };
    
    class TaskSet_fixed_base_walk
    {
    public:
        static ISIRCtrlTaskManager getTask(orcWbiModel& model, orcisir::ISIRController& ctrl);

    };

    class TaskSet_standing
    {
    public:
        static ISIRCtrlTaskManager getTask(orcWbiModel& model, orcisir::ISIRController& ctrl);
    };
#endif // TASKSETTESTS_H

