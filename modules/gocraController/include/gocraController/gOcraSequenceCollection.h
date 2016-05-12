#ifndef GOCRASEQUENCECOLLECTION_H
#define GOCRASEQUENCECOLLECTION_H

#include "gocra/Tasks/gOcraTaskManagerCollectionBase.h"
#include <ocra-yarp/OcraWbiModel.h>

//#include "wocra/Trajectory/wOcraTrajectory.h"

#include <fstream>
#include <vector>
#include <cmath>

class Sequence_InitialPoseHold : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
};

class Sequence_NominalPose : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& gmodel);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
    private:
        // Full posture task
        Eigen::VectorXd                                 q_init;
        Eigen::VectorXd                                 nominal_q;
        double                                          tInitial;
        double                                          tFinal;
        int                                             t_pich_index;
        ocra::Model*                              model;

        gocra::gOcraFullPostureTaskManager*             tmFull;

        //plot data
        int                                             counter;
        std::ofstream                                   jointPositionFile;
        std::ofstream                                   jointVelocityFile;
        std::vector<Eigen::VectorXd>                    jointPositionVector;
        std::vector<Eigen::VectorXd>                    jointVelocityVector;
        std::vector<Eigen::VectorXd>::iterator          jointPositionVectorIt;
        std::vector<Eigen::VectorXd>::iterator          jointVelocityVectorIt;


};

class Sequence_LeftHandReach : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& controller, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& controller, ocra::Model& gmodel);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Segment right hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandRight;
        // Segment head task
        gocra::gOcraSegCartesianTaskManager*           tmHead;
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
        ocra::Model*                             model;
        Eigen::VectorXd                                nominal_q;
        double                                         kp_posture, kd_posture, kp_lh, kd_lh, kp_head, kd_head, a_lh_head, a_head_lh, a_torso_lh, a_torso_head;


        //plot data
        char keyward[256];
        char value[128];
        char a12[128];
        char a21[128];
        char a31[128];
        char a32[128];
        char switch_priority[128];
        int                                            counter;
        int                                            end;
        Eigen::VectorXd                                errCoM,errLH,errQ,vecT;
        std::ofstream                                  resultFile;
        std::ofstream                                  jointPositionFile;
        std::ofstream                                  jointVelocityFile;
        std::vector<Eigen::VectorXd>                   jointPositionVector;
        std::vector<Eigen::VectorXd>                   jointVelocityVector;
        std::vector<Eigen::VectorXd>::iterator         jointPositionVectorIt;
        std::vector<Eigen::VectorXd>::iterator         jointVelocityVectorIt;

};

class Sequence_ComLeftHandReachReplay : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& controller, ocra::Model& gmodel);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;
        // Segment right hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandRight;
        // Segment head task
        gocra::gOcraSegCartesianTaskManager*           tmHead;
        // Segment left hand task
        gocra::gOcraSegCartesianTaskManager*           tmSegCartHandLeft;
        // CoM task
        gocra::gOcraCoMTaskManager*                    tmCoM;
        // Partial posture task for torso
        gocra::gOcraPartialPostureTaskManager*         tmPartialTorso;

        int                                            nt;//nb of active tasks
        Eigen::MatrixXd                                param_priority;
        gocra::GHCJTController*                        ctrl;
        ocra::Model*                             model;
        Eigen::VectorXd                                nominal_q;
        double                                         kp_posture, kd_posture, kp_lh, kd_lh, kp_head, kd_head, a_lh_head, a_head_lh, a_torso_lh, a_torso_head;


        //plot data
        char keyward[256];
        char value[128];
        char a12[128];
        char a21[128];
        char switch_priority[128];
        int                                            counter;
        int                                            end;
        Eigen::VectorXd                                errCoM,errLH,vecT;
        std::ofstream                                  resultFile;

};

class Sequence_LeftRightHandReach : public gocra::gOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);
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
        virtual void doInit(gocra::GHCJTController& ctrl, ocra::Model& model);
        virtual void doUpdate(double time, ocra::Model& state, void** args);

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

        // ocra_yarp::OcraWbiModel& wbiModel;

    private:
        // Full posture task
        gocra::gOcraFullPostureTaskManager*            tmFull;





};

#endif // GOCRASEQUENCECOLLECTION_H
