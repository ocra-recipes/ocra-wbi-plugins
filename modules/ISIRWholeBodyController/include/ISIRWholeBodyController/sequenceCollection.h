#ifndef SEQUENCECOLLECTION_H
#define SEQUENCECOLLECTION_H

#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"
#include "orcisir/Trajectory/ISIRTrajectory.h"


class Sequence_InitialPoseHold : public orcisir::ISIRTaskManagerCollectionBase
{
    protected: 
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args); 
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
};

class Sequence_NominalPose : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        Eigen::VectorXd                                 q_init;
        Eigen::VectorXd                                 nominal_q;
        double                                          tInitial;
        double                                          tFinal;
        int                                             t_pich_index;
        
        orcisir::ISIRFullPostureTaskManager*            tmFull;
};

class Sequence_LeftHandReach : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Segment left hand task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHandLeft;
};

class Sequence_LeftRightHandReach : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
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

class Sequence_CartesianTest : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        orcisir::ISIRSegCartesianTaskManager*           tmLeftHandCart;
        
        Eigen::Vector3d desiredPos;
        int lHandIndex;
};

class Sequence_PoseTest : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        orcisir::ISIRSegPoseTaskManager*                tmLeftHandPose;
        
        Eigen::Displacementd endingDispd;
        int lHandIndex;
};


class Sequence_OrientationTest : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        orcisir::ISIRSegOrientationTaskManager*         tmLeftHandOrient;
        
        Eigen::Rotation3d startingRotd;  
        Eigen::Rotation3d endingRotd; 
        int lHandIndex;
};

  

class Sequence_TrajectoryTrackingTest : public orcisir::ISIRTaskManagerCollectionBase
{
    protected:
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args);
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        orcisir::ISIRSegCartesianTaskManager*           tmLeftHandCart;
        orcisir::ISIRSegPoseTaskManager*                tmLeftHandPose;
        orcisir::ISIRSegOrientationTaskManager*         tmLeftHandOrient;

        // trajectory
        orcisir::ISIRTrajectory*                        leftHandTrajectory;

        Eigen::Displacementd endingDispd;
        Eigen::Rotation3d endingRotd;
        Eigen::Vector3d desiredPos;
        int lHandIndex;
        bool isDisplacementd;
        bool isRotation3d;
        bool isCartesion;
        bool isCartesionWaypoints;
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

#endif // SEQUENCECOLLECTION_H
