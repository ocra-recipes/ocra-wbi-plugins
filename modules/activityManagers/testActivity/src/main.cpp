#include <activityManagerTools/ControlThreadBase.h>
#include <activityManagerTools/TrajectoryThread.h>
#include <activityManagerTools/ControllerConnection.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cout << "No yarp network, quitting\n";
        return 1;
    }

    ControllerConnection ctlCon;

    std::vector<std::string> portNames = ctlCon.getTaskPortNames();

    std::cout << "portNames[4]: " << portNames[4] << std::endl;

    TRAJECTORY_TYPE trajType = GAUSSIAN_PROCESS;

    Eigen::MatrixXd waypoints(3,1);
    waypoints <<    0.1,
                    0.1,
                    0.6;

    TERMINATION_STRATEGY termStrategy = WAIT_DEACTIVATE;

    TrajectoryThread leftHandTrajThread(10, portNames[4], waypoints, trajType, termStrategy);

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
            leftHandTrajThread.setTerminationStrategy(BACK_AND_FORTH);
            if ((yarp::os::Time::now()-startTime)>20.0){
                if(p2){std::cout << "Changing to STOP_THREAD mode:" << std::endl; p2=false;}
                leftHandTrajThread.setTerminationStrategy(STOP_THREAD);
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
