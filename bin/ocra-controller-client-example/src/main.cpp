/*! \file       .cpp
 *  \brief
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

#include <ocra-yarp/ControlThread.h>
#include <ocra-yarp/TrajectoryThread.h>
#include <ocra-yarp/ControllerConnection.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cout << "No yarp network, quitting\n";
        return 1;
    }

    ocra_yarp::ControllerConnection ctlCon;

    std::vector<std::string> portNames = ctlCon.getTaskPortNames();

    std::cout << "portNames[4]: " << portNames[4] << std::endl;

    ocra_yarp::TRAJECTORY_TYPE trajType = ocra_yarp::GAUSSIAN_PROCESS;

    Eigen::MatrixXd waypoints(3,1);
    waypoints <<    0.1,
                    0.1,
                    0.6;

    ocra_yarp::TERMINATION_STRATEGY termStrategy = ocra_yarp::WAIT_DEACTIVATE;

    ocra_yarp::TrajectoryThread leftHandTrajThread(10, portNames[4], waypoints, trajType, termStrategy);

    leftHandTrajThread.setGoalErrorThreshold(0.045);

    std::cout << "Thread started." << std::endl;
    leftHandTrajThread.start();

    bool done=false;
    double startTime=yarp::os::Time::now();

    std::cout << "In the while loop..." << std::endl;

    bool p1, p2, p3;
    p1 = true;
    p2 = true;
    p3 = true;
    while(!done)
    {

        if ((yarp::os::Time::now()-startTime)>10.0){
            if(p1){std::cout << "Changing to BACK_AND_FORTH mode:" << std::endl; p1=false;}
            leftHandTrajThread.setTerminationStrategy(ocra_yarp::BACK_AND_FORTH);
            if ((yarp::os::Time::now()-startTime)>20.0){
                if(p2){std::cout << "Changing to STOP_THREAD mode:" << std::endl; p2=false;}
                leftHandTrajThread.setTerminationStrategy(ocra_yarp::STOP_THREAD);
                if ((yarp::os::Time::now()-startTime)>30.0){
                    if(p3){std::cout << "Finished while loop!" << std::endl; p3=false;}
                    done=true;
                }
            }
        }
    }

    std::cout << "Stopping thread..." << std::endl;
    leftHandTrajThread.stop();

    std::cout << "Module finished." << std::endl;
    return 0;
}


///
// rHandPosStart = rightHandTask->getTaskFramePosition();
// Eigen::Vector3d rHandDisplacement;
//
// rHandDisplacement << 0.05, 0.05, 0.0;
// rHandPosEnd = rHandPosStart + rHandDisplacement;
// rHandPosEnd(2) = 0.41;
//
//
// // Set waypoints and traj
// rightHandTrajectory->setWaypoints(rHandPosStart, rHandPosEnd);
//
// rHandPosStart = rHandPosEnd;
//
// Eigen::Vector3d rHandDisplacement, rHandPosMiddle;
// rHandDisplacement << 0.1, 0.1, 0.05;
// rHandPosMiddle = rHandPosStart + rHandDisplacement;
// rHandDisplacement << 0.15, 0.25, 0.0;
// rHandPosEnd = rHandPosStart + rHandDisplacement;
//
// Eigen::MatrixXd trajWaypoints(3,3);
// trajWaypoints.col(0) << rHandPosStart;
// trajWaypoints.col(1) << rHandPosMiddle;
// trajWaypoints.col(2) << rHandPosEnd;
//
//
// // Set waypoints and traj
// rightHandTrajectory->setMaxVelocity(0.05); // default is 0.1 m/s
// rightHandTrajectory->setWaypoints(trajWaypoints);
//
//
// std::vector<bool> isOptWaypoint(3);
// isOptWaypoint[0] = false;
// isOptWaypoint[1] = true;
// isOptWaypoint[2] = false;
//
// std::vector<Eigen::VectorXi> dofToOptimize(3);
// dofToOptimize[0] = Eigen::VectorXi();
// Eigen::VectorXi tmpVec(3);
// tmpVec << 1,2,3; // X Y Z
// dofToOptimize[1] = tmpVec;
// dofToOptimize[2] = Eigen::VectorXi();
// rightHandTrajectory->setOptimizationWaypoints(isOptWaypoint);
// rightHandTrajectory->setDofToOptimize(dofToOptimize);
//
// optVariables = rightHandTrajectory->getBoptVariables();
