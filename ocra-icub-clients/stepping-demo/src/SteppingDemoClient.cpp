#include "stepping-demo/SteppingDemoClient.h"

SteppingDemoClient::SteppingDemoClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{

}

SteppingDemoClient::~SteppingDemoClient()
{

}

bool SteppingDemoClient::initialize()
{
    // std::string leftFootPortName = clientComs->getTaskPortName("leftFootCartesian");
    // std::string rightFootPortName = clientComs->getTaskPortName("rightFootCartesian");
    // std::string comPortName = clientComs->getTaskPortName("comTask");
    //
    // std::cout << "leftFootPortName: " << leftFootPortName << std::endl;
    // std::cout << "rightFootPortName: " << rightFootPortName << std::endl;
    // std::cout << "comPortName: " << comPortName << std::endl;

    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::WAIT;
    int trajThreadPeriod = 10;


    leftFoot_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(trajThreadPeriod, "LeftFootCartesian", trajType, termStrategy);

    rightFoot_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(trajThreadPeriod, "RightFootCartesian", trajType, termStrategy);

    com_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(trajThreadPeriod, "ComTask", trajType, termStrategy);


    leftFoot_TrajThread->setMaxVelocity(0.02);
    rightFoot_TrajThread->setMaxVelocity(0.02);
    com_TrajThread->setMaxVelocity(0.01);
    com_TrajThread->setGoalErrorThreshold(0.01);

    currentPhase = MOVE_TO_LEFT_SUPPORT;
    isMovingCoM = false;
    
    if (!com_TrajThread->start()) return false;
    if (!leftFoot_TrajThread->start()) return false;
    if (!rightFoot_TrajThread->start()) return false;
    std::cout << "Thread started." << std::endl;

    getInitialValues = true;
    startTime = yarp::os::Time::now();

    LeftFootContact_BackLeft = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_BackLeft");
    LeftFootContact_FrontLeft = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_FrontLeft");
    LeftFootContact_BackRight = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_BackRight");
    LeftFootContact_FrontRight = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_FrontRight");

    RightFootContact_BackLeft = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_BackLeft");
    RightFootContact_FrontLeft = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_FrontLeft");
    RightFootContact_BackRight = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_BackRight");
    RightFootContact_FrontRight = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_FrontRight");


    isInLeftSupportMode = true;
    isInRightSupportMode = true;

    footTrajectoryStarted = false;

    return true;
}

void SteppingDemoClient::release()
{
    com_TrajThread->stop();
    rightFoot_TrajThread->stop();
    leftFoot_TrajThread->stop();
}

void SteppingDemoClient::loop()
{

    currentTime = yarp::os::Time::now();
    // Wait for 2 seconds then start moving. Has something to do with the model states not being correctly updated during the init phase.
    if( (currentTime - startTime) > 2.0 )
    {

        // set intial constants
        if(getInitialValues)
        {
            leftFootHome = getLeftFootPosition();
            rightFootHome = getRightFootPosition();
            comHome = getCoMPosition();

            leftFootTarget = leftFootHome + Eigen::Vector3d(0.05, -0.01, 0.05);
//             leftFootTarget = leftFootHome + Eigen::Vector3d(0.0, 0.0, 0.0);
            rightFootTarget = rightFootHome + Eigen::Vector3d(0.05, 0.01, 0.05);

//             leftFootHome += Eigen::Vector3d(0.0, 0.0, 0.0);
//             rightFootHome += Eigen::Vector3d(0.0, 0.0, 0.0);

            OCRA_INFO(" leftFootHome: " << leftFootHome.transpose());
            OCRA_INFO(" rightFootHome: " << rightFootHome.transpose());
            OCRA_INFO(" comHome: " << comHome.transpose());
            OCRA_INFO(" leftFootTarget: " << leftFootTarget.transpose());
            OCRA_INFO(" rightFootTarget: " << rightFootTarget.transpose());

            getInitialValues = false;
        }

//         std::cout<<"Left foot pose: " << getLeftFootPosition().transpose() << std::endl;
        if(isBalanced() && pauseFinished())
        {
            // std::cout << "Hey I am balanced!" << std::endl;
            switch (currentPhase)
            {
                case MOVE_TO_LEFT_SUPPORT:
                {
                    if(!isMovingCoM) {
                        std::cout << "Moving CoM over left foot." << std::endl;
                        positionCoMOver(LEFT_FOOT_XY);
                        isMovingCoM = true;
                        pauseFor(5.0);
                    }
                    else if (liftFoot(RIGHT_FOOT, true, false)){
                        pauseFor(5.0);
//                         isInLeftSupportMode = true;
//                         isInRightSupportMode = false;
                        currentPhase = MOVE_TO_DOUBLE_SUPPORT;
                        isMovingCoM = false;
                    }

                }break;

                case MOVE_TO_RIGHT_SUPPORT:
                {
                    if(!isMovingCoM) {
                        std::cout << "Moving CoM over right foot." << std::endl;
                        positionCoMOver(RIGHT_FOOT_XY);
                        isMovingCoM = true;
                        pauseFor(5.0);
                    }
                    else if (liftFoot(LEFT_FOOT, false, true)){
                        pauseFor(5.0);
//                         isInLeftSupportMode = false;
//                         isInRightSupportMode = true;
                        currentPhase = MOVE_TO_DOUBLE_SUPPORT;
                        isMovingCoM = false;
                    }
                }break;

                case MOVE_TO_DOUBLE_SUPPORT:
                {
                    if(isInLeftSupportMode && !isInRightSupportMode) {
                        if (setDownFoot(RIGHT_FOOT)) {
                            isInLeftSupportMode = false;
                            nextPhase = MOVE_TO_RIGHT_SUPPORT;
                            pauseFor(5.0);
                        }
                    }
                    else if(isInRightSupportMode && !isInLeftSupportMode) {
                        if (setDownFoot(LEFT_FOOT)) {
                            isInRightSupportMode = false;
                            nextPhase = MOVE_TO_LEFT_SUPPORT;
                            pauseFor(5.0);
                        }
                    }
                    else
                    {
                        if(!isMovingCoM) {
                            std::cout << "Moving CoM over between feet." << std::endl;
                            positionCoMOver(CENTERED_BETWEEN_FEET_XY);
                            isMovingCoM = true;
                            pauseFor(5.0);
                        }
                        else
                        {
                            pauseFor(5.0);
                            isInLeftSupportMode = true;
                            isInRightSupportMode = true;
                            currentPhase = nextPhase;
                            isMovingCoM = false;
                        }
                    }
                }break;
            }
        }
    }
}

Eigen::Vector3d SteppingDemoClient::getLeftFootPosition()
{
    return model->getSegmentPosition(model->getSegmentIndex("l_sole")).getTranslation();
}

Eigen::Vector3d SteppingDemoClient::getRightFootPosition()
{
    return model->getSegmentPosition(model->getSegmentIndex("r_sole")).getTranslation();
}

Eigen::Vector3d SteppingDemoClient::getCoMPosition()
{
    return model->getCoMPosition();
}

void SteppingDemoClient::positionCoMOver(COM_SUPPORT_POSITION newSupportPos)
{
    switch (newSupportPos)
    {
        case LEFT_FOOT_XY:
        {
            newCoMGoalPosition = getLeftFootPosition();
//             std::cout << "newCoMGoalPosition: " << newCoMGoalPosition.transpose() << std::endl;

        }break;
        case RIGHT_FOOT_XY:
        {
            newCoMGoalPosition = getRightFootPosition();
        }break;
        case CENTERED_BETWEEN_FEET_XY:
        {
            Eigen::Vector3d leftPos = getLeftFootPosition();
            Eigen::Vector3d rightPos = getRightFootPosition();

            newCoMGoalPosition = (leftPos + rightPos) / 2.0;
        }break;

        default:
        {
            newCoMGoalPosition = getCoMPosition();
            std::cout << "Error: doing nothing because you did not provide a correct CoM position." << std::endl;
        }break;
    }


    newCoMGoalPosition(2) = getCoMPosition()(2);

    std::cout << "newCoMGoalPosition: " << newCoMGoalPosition.transpose() << std::endl;

    com_TrajThread->setTrajectoryWaypoints(newCoMGoalPosition.head(2));
}


bool SteppingDemoClient::isBalanced()
{

    double comVelNorm = model->getCoMVelocity().norm();
    double jointVelNorm = model->getJointVelocities().norm();

    // std::cout << "--------------------" << std::endl;
    // std::cout << "comVelNorm: " << comVelNorm << std::endl;
    // std::cout << "jointVelNorm: " << jointVelNorm << std::endl;

    double comVelThreshold = 0.01;
    double jointThreshold = 0.07;

    return ( (comVelNorm <= comVelThreshold) && (jointVelNorm <= jointThreshold) );

}


void SteppingDemoClient::deactivateFootContacts(FOOT_CONTACTS foot)
{
    switch (foot) {
        case LEFT_FOOT:
        {
            bool res;
            res = LeftFootContact_BackLeft->deactivate();
            res &= LeftFootContact_FrontLeft->deactivate();
            res &= LeftFootContact_BackRight->deactivate();
            res &= LeftFootContact_FrontRight->deactivate();
            if(res) {
                std::cout << "Deactivated left foot contacts." << std::endl;
            }
        }break;

        case RIGHT_FOOT:
        {
            bool res;
            res = RightFootContact_BackLeft->deactivate();
            res &= RightFootContact_FrontLeft->deactivate();
            res &= RightFootContact_BackRight->deactivate();
            res &= RightFootContact_FrontRight->deactivate();
            if(res) {
                std::cout << "Deactivated right foot contacts." << std::endl;
            }
        }break;

        default:
        break;
    }
}

void SteppingDemoClient::activateFootContacts(FOOT_CONTACTS foot)
{
    switch (foot) {
        case LEFT_FOOT:
        {
            bool res;
            res = LeftFootContact_BackLeft->activate();
            res &= LeftFootContact_FrontLeft->activate();
            res &= LeftFootContact_BackRight->activate();
            res &= LeftFootContact_FrontRight->activate();
            if(res) {
                std::cout << "Activated left foot contacts." << std::endl;
            } else {
                OCRA_ERROR("One or more contacts could not be activated");
            }
        }break;

        case RIGHT_FOOT:
        {
            bool res;
            res = RightFootContact_BackLeft->activate();
            res &= RightFootContact_FrontLeft->activate();
            res &= RightFootContact_BackRight->activate();
            res &= RightFootContact_FrontRight->activate();
            if(res) {
                std::cout << "Activated right foot contacts." << std::endl;
            } else {
                OCRA_ERROR("One or more contacts could not be activated");
            }
        }break;

        default:
        break;
    }
}


bool SteppingDemoClient::isFootInContact(FOOT_CONTACTS foot)
{
    double foot_z;
    switch (foot) {
        case LEFT_FOOT:
        {
            foot_z = getLeftFootPosition()(2);
        }break;

        case RIGHT_FOOT:
        {
            foot_z = getRightFootPosition()(2);
        }break;

        default:
        break;
    }

    double footContactRealeaseThreshold = 0.005;

    if (foot_z <= footContactRealeaseThreshold) {
        activateFootContacts(foot);
        return true;
    } else {
        return false;
    }

}

void SteppingDemoClient::pauseFor(double _pauseDuration)
{
    pauseTriggerTime = yarp::os::Time::now();
    isPausing = true;
    pauseDuration = _pauseDuration;
}

bool SteppingDemoClient::pauseFinished()
{
    if(isPausing)
    {
        if ((currentTime - pauseTriggerTime) >= pauseDuration) {
            isPausing = false;
            return true;
        } else {
            return false;
        }
    }
    else{return true;}
}

bool SteppingDemoClient::liftFoot(FOOT_CONTACTS foot)
{
    double delayTime = 0.2;
    switch (foot) {
        case LEFT_FOOT:
        {
            if (!footTrajectoryStarted) {
                std::cout << "Setting left foot trajectory." << std::endl;
                leftFoot_TrajThread->setGoalErrorThreshold(0.01);
                 // Switch fixedLink if odometry is active on the server-side
                 // before the foot trajectory is set and the contact is deactivated
                if ( !this->changeFixedLink("r_sole", isInLeftSupportMode, isInRightSupportMode) ) {
                    OCRA_ERROR("Fixed link could not be changed to r_sole");
                    return false;
                }
                    OCRA_INFO("FIXED LINK WAS SWITCHED TO THE RIGHT FOOT");
                leftFoot_TrajThread->setTrajectoryWaypoints(leftFootTarget);
                yarp::os::Time::delay(delayTime);
                deactivateFootContacts(LEFT_FOOT);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(leftFoot_TrajThread->goalAttained()) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        case RIGHT_FOOT:
        {
            if (!footTrajectoryStarted) {
                OCRA_INFO("Setting right foot trajectory.");
                rightFoot_TrajThread->setGoalErrorThreshold(0.01);
                rightFoot_TrajThread->setTrajectoryWaypoints(rightFootTarget);
                yarp::os::Time::delay(delayTime);
                deactivateFootContacts(RIGHT_FOOT);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(rightFoot_TrajThread->goalAttained()) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        default:
        break;
    }

}

bool SteppingDemoClient::liftFoot(FOOT_CONTACTS foot, bool isLeftFootInContact, bool isRightFootInContact)
{
    this->isInLeftSupportMode = isLeftFootInContact;
    this->isInRightSupportMode = isRightFootInContact;
    double delayTime = 0.2;
    switch (foot) {
        case LEFT_FOOT:
        {
            if (!footTrajectoryStarted) {
                OCRA_INFO("Setting left foot trajectory.");
                leftFoot_TrajThread->setGoalErrorThreshold(0.01);
                 // Switch fixedLink if odometry is active on the server-side
                 // before the foot trajectory is set and the contact is deactivated
                this->changeFixedLink("r_sole", isLeftFootInContact, isRightFootInContact);
                OCRA_INFO("FIXED LINK WAS SWITCHED TO THE RIGHT FOOT");
                leftFoot_TrajThread->setTrajectoryWaypoints(leftFootTarget);
                yarp::os::Time::delay(delayTime);
                deactivateFootContacts(LEFT_FOOT);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(leftFoot_TrajThread->goalAttained()) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        case RIGHT_FOOT:
        {
            if (!footTrajectoryStarted) {
                std::cout << "Setting right foot trajectory." << std::endl;
                rightFoot_TrajThread->setGoalErrorThreshold(0.01);
                rightFoot_TrajThread->setTrajectoryWaypoints(rightFootTarget);
                yarp::os::Time::delay(delayTime);
                deactivateFootContacts(RIGHT_FOOT);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(rightFoot_TrajThread->goalAttained()) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        default:
        break;
    }

}


bool SteppingDemoClient::setDownFoot(FOOT_CONTACTS foot)
{
    switch (foot) {
        case LEFT_FOOT:
        {
            if (!footTrajectoryStarted) {
                OCRA_INFO("Setting left foot trajectory.");
                leftFoot_TrajThread->setGoalErrorThreshold(0.008);
                leftFoot_TrajThread->setTrajectoryWaypoints(leftFootHome);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(isFootInContact(LEFT_FOOT)) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        case RIGHT_FOOT:
        {
            if (!footTrajectoryStarted) {
                OCRA_INFO("Setting right foot trajectory.");
                rightFoot_TrajThread->setGoalErrorThreshold(0.008);
                rightFoot_TrajThread->setTrajectoryWaypoints(rightFootHome);
                footTrajectoryStarted = true;
                return false;
            } else {
                if(isFootInContact(RIGHT_FOOT)) {
                    footTrajectoryStarted = false;
                    return true;
                } else {
                    return false;
                }
            }
        }break;

        default:
        break;
    }

}
