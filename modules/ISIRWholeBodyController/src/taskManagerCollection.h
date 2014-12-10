#ifndef TASKMANAGERCOLLECTION_H
#define TASKMANAGERCOLLECTION_H

#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"

class TaskCollection_InitialPoseHold : public orcisir::ISIRTaskManagerCollectionBase
{
    protected: 
        virtual void doInit(orcisir::ISIRController& ctrl, orc::Model& model);
        virtual void doUpdate(double time, orc::Model& state, void** args); 
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
};

class TaskCollection_NominalPose : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orc::Model& model);
        virtual void doUpdate(double time, orc::Model& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
};

class TaskCollection_LeftHandReach : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orc::Model& model);
        virtual void doUpdate(double time, orc::Model& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Segment left hand task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHandLeft;
};

class TaskCollection_LeftRightHandReach : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orc::Model& model);
        virtual void doUpdate(double time, orc::Model& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial posture task for torso
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // CoM task for torso
        orcisir::ISIRCoMTaskManager*                    tmCoM;
        // Segment left hand task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHandLeft;
        // Segment right hand task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHandRight;
};



/*
        // Partial torso posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Left foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactRight;
        // CoM task
        orcisir::ISIRCoMTaskManager*                    tmCoM;
        // Segment head task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHead;
        // Segment left foot task
        orcisir::ISIRSegPoseTaskManager*           tmSegCartFootLeft;
        // Segment right foot task
        orcisir::ISIRSegPoseTaskManager*           tmSegCartFootRight;
*/

#endif // TASKMANAGERCOLLECTION_H
