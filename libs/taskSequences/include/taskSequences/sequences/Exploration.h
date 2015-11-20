#ifndef EXPLORATION_H
#define EXPLORATION_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"
#include "wocra/Trajectory/wOcraExperimentalTrajectory.h"
#include "yarp/os/all.h"

// #include "wocra/Trajectory/wOcraTrajectory.h"

// namespace sequence {

    class Exploration : public wocra::wOcraTaskSequenceBase
    {
        public:
            ~Exploration();
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);

        private:
            yarp::os::Network yarp;
            yarp::os::Port l_hand_port, l_hand_target_port, r_hand_port, r_hand_target_port;


            wocra::wOcraExperimentalTrajectory* leftHandTrajectory;
            wocra::wOcraExperimentalTrajectory* rightHandTrajectory;


            Eigen::VectorXd currentDesiredPosition_leftHand;
            Eigen::VectorXd currentDesiredPosition_rightHand;

            int lHandIndex, rHandIndex;

            wocra::wOcraVariableWeightsTaskManager* leftHandTask;
            wocra::wOcraVariableWeightsTaskManager* rightHandTask;

            double maxVariance;
            double resetTimeLeft, resetTimeRight;

            bool attainedGoal(wocra::wOcraModel& state, int segmentIndex);

            Eigen::MatrixXd desiredPosVelAcc_leftHand, desiredPosVelAcc_rightHand;
            Eigen::VectorXd desiredVariance_leftHand, desiredVariance_rightHand, desiredWeights_leftHand, desiredWeights_rightHand;

            Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);

            void generateNewWaypoints(wocra::wOcraModel& state, int segmentIndex);

            Eigen::VectorXd generateTarget(int segmentIndex);

            bool initTrigger;

            Eigen::Array3d varianceThresh;

    };

// }


#endif
