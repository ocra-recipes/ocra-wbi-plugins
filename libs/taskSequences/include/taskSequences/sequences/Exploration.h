#ifndef EXPLORATION_H
#define EXPLORATION_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"
#include "ocra/control/Trajectory/ExperimentalTrajectory.h"
#include "yarp/os/all.h"

// #include "ocra/control/Trajectory/Trajectory.h"

// namespace sequence {

    class Exploration : public ocra::TaskSequence
    {
        public:
            ~Exploration();
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);

        private:
            yarp::os::Network yarp;
            yarp::os::Port l_hand_port, l_hand_target_port, r_hand_port, r_hand_target_port;


            ocra::ExperimentalTrajectory* leftHandTrajectory;
            ocra::ExperimentalTrajectory* rightHandTrajectory;


            Eigen::VectorXd currentDesiredPosition_leftHand;
            Eigen::VectorXd currentDesiredPosition_rightHand;

            int lHandIndex, rHandIndex;

            ocra::VariableWeightsTaskManager* leftHandTask;
            ocra::VariableWeightsTaskManager* rightHandTask;

            double maxVariance;
            double resetTimeLeft, resetTimeRight;

            bool attainedGoal(ocra::Model& state, int segmentIndex);

            Eigen::MatrixXd desiredPosVelAcc_leftHand, desiredPosVelAcc_rightHand;
            Eigen::VectorXd desiredVariance_leftHand, desiredVariance_rightHand, desiredWeights_leftHand, desiredWeights_rightHand;

            Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);

            void generateNewWaypoints(ocra::Model& state, int segmentIndex);

            Eigen::VectorXd generateTarget(int segmentIndex);

            bool initTrigger;

            Eigen::Array3d varianceThresh;

    };

// }


#endif
