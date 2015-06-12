#ifndef GOCRASEQUENCECOLLECTION_H
#define GOCRASEQUENCECOLLECTION_H

#include "gocra/Tasks/gOcraTaskManagerCollectionBase.h"
//#include "wocra/Trajectory/wOcraTrajectory.h"

#include <fstream>

class Sequence_InitialPoseHold : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
};

class Sequence_NominalPose : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        Eigen::VectorXd                                 q_init;
        Eigen::VectorXd                                 nominal_q;
        double                                          tInitial;
        double                                          tFinal;
        int                                             t_pich_index;

        gocra::gOcraFullPostureTaskManager*            tmFull;
};

class Sequence_LeftHandReach : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& controller, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandLeft;

        int                                            nt;//nb of active tasks
        double                                         tInitial;
        bool                                           tInitialSet;
        double                                         tSwitch;//start priority switch
        double                                         tFinal;//stop priority switch
        double                                         switchDuration;
        double                                         oneToZero;
        double                                         zeroToOne;
        Eigen::MatrixXd                                param_priority;
        gocra::GHCJTController*                        ctrl;
};

class Sequence_ComLeftHandReach : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& controller, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Segment right hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandRight;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandLeft;
        // CoM task
        gocra::gOcraCoMTaskManager*                    tmCoM;
        // Partial posture task for torso
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment right foot task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartFootRight;
        // Segment left foot task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartFootLeft;
        int                                            nt;//nb of active tasks
        double                                         tInitial;
        bool                                           tInitialSet;
        double                                         tSwitch;//start priority switch
        double                                         tFinal;//stop priority switch
        double                                         switchDuration;
        double                                         oneToZero;
        double                                         zeroToOne;
        Eigen::MatrixXd                                param_priority;
        gocra::GHCJTController*                        ctrl;
        gocra::gOcraModel*                             model;
        Eigen::VectorXd                                nominal_q;

        //plot data
        int                                            counter;
        int                                            end;
        Eigen::VectorXd                                errCoM,errLH,errQ,vecT;
        std::ofstream                                  resultFile;
};

class Sequence_LeftRightHandReach : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& gmodel);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Partial posture task for torso
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // CoM task for torso
        gocra::gOcraCoMTaskManager*                    tmCoM;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandLeft;
        // Segment right hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandRight;
};

class Sequence_CartesianTest : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmLeftHandCart;

        Eigen::Vector3d desiredPos;
        int lHandIndex;
};

class Sequence_PoseTest : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        gocra::gOcraSegPoseTaskManager*                tmLeftHandPose;

        Eigen::Displacementd endingDispd;
        int lHandIndex;
};


class Sequence_OrientationTest : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        gocra::gOcraSegOrientationTaskManager*         tmLeftHandOrient;

        Eigen::Rotation3d startingRotd;
        Eigen::Rotation3d endingRotd;
        int lHandIndex;
};


/*
class Sequence_TrajectoryTrackingTest : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Partial posture task
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmLeftHandCart;
        gocra::gOcraSegPoseTaskManager*                tmLeftHandPose;
        gocra::gOcraSegOrientationTaskManager*         tmLeftHandOrient;

        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandRight;

        // trajectory
        gocra::gOcraTrajectory*                        leftHandTrajectory;

        Eigen::Displacementd endingDispd;
        Eigen::Rotation3d endingRotd;
        Eigen::Vector3d desiredPos;
        int lHandIndex;
        bool isDisplacementd;
        bool isRotation3d;
        bool isCartesion;
        bool isCartesionWaypoints;
};

*/
class Sequence_FloatingBaseEstimationTests : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
};



/*
        // Partial torso posture task
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;
        // Left foot contact task
        gocra::gOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        gocra::gOcraContactSetTaskManager*             tmFootContactRight;
        // CoM task
        gocra::gOcraCoMTaskManager*                    tmCoM;
        // Segment head task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHead;
        // Segment left foot task
        gocra::gOcraSegPoseTaskManager*           tmSegCartFootLeft;
        // Segment right foot task
        gocra::gOcraSegPoseTaskManager*           tmSegCartFootRight;
*/


class Sequence_JointTest : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model);
        virtual void doUpdate(double time, gocra::gOcraModel& state, void** args);

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
        gocra::gOcraFullPostureTaskManager*            tmFull;





};

#endif // GOCRASEQUENCECOLLECTION_H
