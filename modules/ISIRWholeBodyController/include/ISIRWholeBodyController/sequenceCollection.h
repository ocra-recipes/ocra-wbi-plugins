#ifndef SEQUENCECOLLECTION_H
#define SEQUENCECOLLECTION_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "wocra/Trajectory/wOcraTrajectory.h"

class Sequence_FixedBaseMinimalTasks : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
};

class Sequence_FloatingBaseMinimalTasks: public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};


class Sequence_InitialPoseHold : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
};

class Sequence_NominalPose : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        Eigen::VectorXd                                 q_init;
        Eigen::VectorXd                                 nominal_q;
        double                                          tInitial;
        double                                          tFinal;
        int                                             t_pich_index;

        wocra::wOcraFullPostureTaskManager*            tmFull;
};

class Sequence_LeftHandReach : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Segment left hand task
        wocra::wOcraSegCartesianTaskManager*           tmSegCartHandLeft;
};

class Sequence_LeftRightHandReach : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial posture task for torso
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // CoM task for torso
        wocra::wOcraCoMTaskManager*                    tmCoM;
        // Segment left hand task
        wocra::wOcraSegCartesianTaskManager*           tmSegCartHandLeft;
        // Segment right hand task
        wocra::wOcraSegCartesianTaskManager*           tmSegCartHandRight;
};

class Sequence_CartesianTest : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        wocra::wOcraSegCartesianTaskManager*           tmLeftHandCart;

        Eigen::Vector3d desiredPos;
        int lHandIndex;
};

class Sequence_PoseTest : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        wocra::wOcraSegPoseTaskManager*                tmLeftHandPose;

        Eigen::Displacementd endingDispd;
        int lHandIndex;
};


class Sequence_OrientationTest : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        wocra::wOcraSegOrientationTaskManager*         tmLeftHandOrient;

        Eigen::Rotation3d startingRotd;
        Eigen::Rotation3d endingRotd;
        int lHandIndex;
};



class Sequence_TrajectoryTrackingTest : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        wocra::wOcraSegCartesianTaskManager*           tmLeftHandCart;
        wocra::wOcraSegPoseTaskManager*                tmLeftHandPose;
        wocra::wOcraSegOrientationTaskManager*         tmLeftHandOrient;

        wocra::wOcraSegCartesianTaskManager*           tmSegCartHandRight;

        // trajectory
        wocra::wOcraTrajectory*                        leftHandTrajectory;

        Eigen::Displacementd endingDispd;
        Eigen::Rotation3d endingRotd;
        Eigen::Vector3d desiredPos;
        int lHandIndex;
        bool isDisplacementd;
        bool isRotation3d;
        bool isCartesion;
        bool isCartesionWaypoints;
};


class Sequence_FloatingBaseEstimationTests : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
};



/*
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialTorso;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // CoM task
        wocra::wOcraCoMTaskManager*                    tmCoM;
        // Segment head task
        wocra::wOcraSegCartesianTaskManager*           tmSegCartHead;
        // Segment left foot task
        wocra::wOcraSegPoseTaskManager*           tmSegCartFootLeft;
        // Segment right foot task
        wocra::wOcraSegPoseTaskManager*           tmSegCartFootRight;
*/


class Sequence_JointTest : public wocra::wOcraTaskSequenceBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);

        Eigen::VectorXd q_init;
        Eigen::VectorXd q_des;
        Eigen::VectorXd jointMin;
        Eigen::VectorXd jointMax;
        int nDoF;
        int jIndex;
        double taskErr;
        bool goToMin;
        bool goToMax;


        std::string jointNames [25];
        int counter;

        // ocraWbiModel& wbiModel;

    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;





};

#endif // SEQUENCECOLLECTION_H
