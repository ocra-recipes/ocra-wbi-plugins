/*! \file       TrajectoryThread.cpp
 *  \brief      A thread for launching trajectory generators.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-yarp.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ocra-yarp/TrajectoryThread.h>

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

using namespace ocra_yarp;


TrajectoryThread::TrajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE trajectoryType, const TERMINATION_STRATEGY _terminationStrategy):
ControlThread(period, taskPortName),
userWaypoints(waypoints),
trajType(trajectoryType),
terminationStrategy(_terminationStrategy),
printWaitingNoticeOnce(true),
errorThreshold(0.03),
useVarianceModulation(true),
deactivationDelay(0.0),
deactivationTimeout(5.0),
deactivationLatch(false)
{
    setThreadType("TrajectoryThread");

    switch (trajType)
    {
        case MIN_JERK:
            trajectory = new ocra::MinimumJerkTrajectory();
            break;
        case LIN_INTERP:
            trajectory = new ocra::LinearInterpolationTrajectory();
            break;
        case GAUSSIAN_PROCESS:
            #if USING_SMLT
            trajectory = new ocra::GaussianProcessTrajectory();
            #else
            std::cout << "You need the SMLT libs to use GAUSSIAN_PROCESS type trajectories. I'm gonna make you a MIN_JERK instead." << std::endl;
            trajType = MIN_JERK;
            trajectory = new ocra::MinimumJerkTrajectory();
            #endif
            break;
    }
}

bool TrajectoryThread::ct_threadInit()
{
    if (trajType==GAUSSIAN_PROCESS)
    {
        desiredVariance = Eigen::VectorXd::Ones(weightDimension);
        varianceThresh = Eigen::ArrayXd::Constant(weightDimension, VAR_THRESH);
    }

    return setTrajectoryWaypoints(userWaypoints);
}

void TrajectoryThread::ct_threadRelease()
{
    delete trajectory;
    trajectory = NULL;
    std::cout<< "\nTrajectoryThread: Trajectory thread finished.\n";
}

void TrajectoryThread::ct_run()
{
    if (goalAttained() || deactivationLatch)
    {
        switch (terminationStrategy)
        {
            case BACK_AND_FORTH:
                flipWaypoints();
                setTrajectoryWaypoints(allWaypoints, true);
                break;
            case STOP_THREAD:
                stop();
                break;
            case STOP_THREAD_DEACTIVATE:
                if(deactivateTask()){
                    stop();
                }else{
                    std::cout << "[WARNING] Trajectory id = "<< ControlThread::threadId << " for task: " << originalTaskParams.name << " has attained its goal state, but cannot be deactivated." << std::endl;
                    yarp::os::Time::delay(1.0); // try again in one second.
                    deactivationDelay += 1.0;
                    if(deactivationDelay >= deactivationTimeout){
                        std::cout << "[WARNING] Deactivation timeout." << std::endl;
                        stop();
                    }
                }
                break;
            case WAIT:
                if (printWaitingNoticeOnce) {
                    std::cout << "Trajectory id = "<< ControlThread::threadId << " for task: " << originalTaskParams.name << " has attained its goal state. Awaiting new commands." << std::endl;
                    printWaitingNoticeOnce = false;
                }
                break;
            case WAIT_DEACTIVATE:
                if (printWaitingNoticeOnce) {
                    if(deactivateTask()){
                        std::cout << "Trajectory id = "<< ControlThread::threadId << " for task: " << originalTaskParams.name << " has attained its goal state. Deactivating task and awaiting new commands." << std::endl;
                        printWaitingNoticeOnce = false;
                        deactivationLatch = true;
                    }else{
                        std::cout << "Trajectory id = "<< ControlThread::threadId << " for task: " << originalTaskParams.name << " has attained its goal state and is awaiting new commands. [WARNING] Could not deactivate the task." << std::endl;
                        yarp::os::Time::delay(1.0); // try again in one second.
                        deactivationDelay += 1.0;
                        if(deactivationDelay >= deactivationTimeout){
                            printWaitingNoticeOnce = false;
                            std::cout << "[WARNING] Deactivation timeout." << std::endl;
                        }
                    }
                }
                break;
        }
    }
    else{
        if (!currentTaskParams.isActive) {
            activateTask();
        }

        desStateBottle.clear();
        if (trajType==GAUSSIAN_PROCESS)
        {
            Eigen::MatrixXd desiredState_tmp;
            trajectory->getDesiredValues(yarp::os::Time::now(), desiredState_tmp, desiredVariance);
            desiredState << desiredState_tmp;


            for(int i=0; i<desiredState.size(); i++)
            {
                desStateBottle.addDouble(desiredState(i));
            }
            #if USING_SMLT
            if(useVarianceModulation)
            {
                Eigen::VectorXd desiredWeights = varianceToWeights(desiredVariance);
                for(int i=0; i<desiredWeights.size(); i++)
                {
                    desStateBottle.addDouble(desiredWeights(i));
                }
            }
            #endif
        }
        else
        {
            desiredState << trajectory->getDesiredValues(yarp::os::Time::now());
            for(int i=0; i<desiredState.size(); i++)
            {
                desStateBottle.addDouble(desiredState(i));
            }
        }


        outputPort.write(desStateBottle);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd TrajectoryThread::varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta)
{
    desiredVariance /= maximumVariance;
    desiredVariance = desiredVariance.array().min(varianceThresh); //limit desiredVariance to 0.99 maximum
    Eigen::VectorXd desiredWeights = (Eigen::VectorXd::Ones(desiredVariance.rows()) - desiredVariance) / beta;
    return desiredWeights;
}

bool TrajectoryThread::goalAttained()
{
    return (goalStateVector - getCurrentState().head(weightDimension)).norm() <= errorThreshold;
}

void TrajectoryThread::flipWaypoints()
{
    int nCols = allWaypoints.cols();
    Eigen::MatrixXd tmp(allWaypoints.rows(), nCols);

    int c_tmp = nCols - 1;
    for(int c=0; c<nCols; c++)
    {
        tmp.col(c_tmp) = allWaypoints.col(c);
        c_tmp--;
    }
    allWaypoints = tmp;
    goalStateVector = allWaypoints.rightCols(1);
}

bool TrajectoryThread::setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint)
{
    Eigen::MatrixXd _userWaypoints = userWaypoints; // Copy waypoints first in case we use allWaypoints as an arg.
    if (weightDimension==_userWaypoints.rows())
    {
        if(containsStartingWaypoint)
        {
            allWaypoints = _userWaypoints;
        }
        else // Add starting waypoint
        {
            allWaypoints = Eigen::MatrixXd(weightDimension, _userWaypoints.cols()+1);

            startStateVector = getCurrentState();
            desiredState = Eigen::VectorXd::Zero(startStateVector.size());


            allWaypoints.col(0) << currentStateVector.head(weightDimension);
            for(int i=0; i<_userWaypoints.cols(); i++)
            {
                allWaypoints.col(i+1) << _userWaypoints.col(i);
            }
        }

        goalStateVector = allWaypoints.rightCols(1);

        trajectory->setWaypoints(allWaypoints);

        #if USING_SMLT
        if (trajType==GAUSSIAN_PROCESS)
        {
            maximumVariance = dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->getMaxVariance();
        }
        #endif

        printWaitingNoticeOnce=true;
        deactivationLatch = false;

        return true;
    }
    else
    {
        std::cout << "[ERROR](TrajectoryThread::setTrajectoryWaypoints): The dimension (# DOF) of the waypoints you provided, " << _userWaypoints.rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
        return false;
    }
}


bool TrajectoryThread::setDisplacement(double dispDouble)
{
    return setDisplacement(Eigen::VectorXd::Constant(weightDimension, dispDouble));
}

bool TrajectoryThread::setDisplacement(const Eigen::VectorXd& displacementVector)
{
    if (weightDimension == displacementVector.rows())
    {
        startStateVector = getCurrentState();
        Eigen::MatrixXd tmpWaypoints = Eigen::MatrixXd::Zero(weightDimension, 2);
        tmpWaypoints.col(0) << startStateVector;
        tmpWaypoints.col(1) << startStateVector + displacementVector;
        setTrajectoryWaypoints(tmpWaypoints, true);
        return true;
    }
    else{
        return false;
    }
}



#if USING_SMLT
void TrajectoryThread::setMeanWaypoints(std::vector<bool>& isMeanWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setMeanWaypoints(isMeanWaypoint);
}

void TrajectoryThread::setVarianceWaypoints(std::vector<bool>& isVarWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setVarianceWaypoints(isVarWaypoint);
}

void TrajectoryThread::setOptimizationWaypoints(std::vector<bool>& isOptWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setOptimizationWaypoints(isOptWaypoint);
}

void TrajectoryThread::setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setDofToOptimize(dofToOptimize);
}

Eigen::VectorXd TrajectoryThread::getBayesianOptimizationVariables()
{
    return dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->getBoptVariables();
}
#endif
