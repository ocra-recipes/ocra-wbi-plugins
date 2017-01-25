#include "walking-client/StepController.h"


StepController::StepController(int periodms, ocra::Model::Ptr model):
_model(model),
_period(periodms) {}

StepController::~StepController() {

}

bool StepController::initialize() {
    // Specify type of feet trajectory
    ocra_recipes::TRAJECTORY_TYPE trajType = ocra_recipes::MIN_JERK;
    ocra_recipes::TERMINATION_STRATEGY termStrategy = ocra_recipes::WAIT;

    // Create trajectory objects
    _leftFoot_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(_period, "LeftFootCartesian", trajType, termStrategy);
    _rightFoot_TrajThread = std::make_shared<ocra_recipes::TrajectoryThread>(_period, "RightFootCartesian", trajType, termStrategy);

    //TODO: We don't really want this! The speed should be given based on the time in which the step should be performed.
    _leftFoot_TrajThread->setMaxVelocity(0.02);
    _rightFoot_TrajThread->setMaxVelocity(0.02);

    // Start trajectory threads
    if (!_leftFoot_TrajThread->start()) return false;
    if (!_rightFoot_TrajThread->start()) return false;
    OCRA_INFO("Feet trajectory threads started.");

    // Create task connection objects
    _LeftFootContact_BackLeft = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_BackLeft");
    _LeftFootContact_FrontLeft = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_FrontLeft");
    _LeftFootContact_BackRight = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_BackRight");
    _LeftFootContact_FrontRight = std::make_shared<ocra_recipes::TaskConnection>("LeftFootContact_FrontRight");

    _RightFootContact_BackLeft = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_BackLeft");
    _RightFootContact_FrontLeft = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_FrontLeft");
    _RightFootContact_BackRight = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_BackRight");
    _RightFootContact_FrontRight = std::make_shared<ocra_recipes::TaskConnection>("RightFootContact_FrontRight");

    _leftFoot_TrajThread->setGoalErrorThreshold(0.01);
    _rightFoot_TrajThread->setGoalErrorThreshold(0.01);
    return true;
}

void StepController::deactivateFeetContacts(FOOT foot) {
    switch (foot) {
        case LEFT_FOOT:
        {
            bool res;
            res = _LeftFootContact_BackLeft->deactivate();
            res &= _LeftFootContact_FrontLeft->deactivate();
            res &= _LeftFootContact_BackRight->deactivate();
            res &= _LeftFootContact_FrontRight->deactivate();
            if(res) {
                std::cout << "Deactivated left foot contacts." << std::endl;
            }
        }break;

        case RIGHT_FOOT:
        {
            bool res;
            res = _RightFootContact_BackLeft->deactivate();
            res &= _RightFootContact_FrontLeft->deactivate();
            res &= _RightFootContact_BackRight->deactivate();
            res &= _RightFootContact_FrontRight->deactivate();
            if(res) {
                std::cout << "Deactivated right foot contacts." << std::endl;
            }
        }break;

        default:
            break;
    }
}

void StepController::activateFeetContacts(FOOT foot) {
    switch (foot) {
        case LEFT_FOOT:
        {
            bool res;
            res = _LeftFootContact_BackLeft->activate();
            res &= _LeftFootContact_FrontLeft->activate();
            res &= _LeftFootContact_BackRight->activate();
            res &= _LeftFootContact_FrontRight->activate();
            if(res) {
                std::cout << "Activated left foot contacts." << std::endl;
            } else {
                OCRA_ERROR("One or more contacts could not be activated");
            }
        }break;

        case RIGHT_FOOT:
        {
            bool res;
            res = _RightFootContact_BackLeft->activate();
            res &= _RightFootContact_FrontLeft->activate();
            res &= _RightFootContact_BackRight->activate();
            res &= _RightFootContact_FrontRight->activate();
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

bool StepController::doStepWithMaxVelocity(FOOT foot, Eigen::Vector3d target, double stepHeight) {
    Eigen::Vector3d midPoint; midPoint.setZero();
    Eigen::MatrixXd wayPoints(3,2);

    // Pass only the midpoint and the target point. The current position should be included by default.
    this->computeMidPoint(foot, target, stepHeight, midPoint);
    wayPoints.col(0) = midPoint;
    wayPoints.col(1) = target;
    _leftFoot_TrajThread->setTrajectoryWaypoints(wayPoints);

    return true;
}

void StepController::computeMidPoint(FOOT foot, Eigen::Vector3d target, double stepHeight, Eigen::Vector3d & midPoint) {
    switch (foot) {
        case LEFT_FOOT:
            _leftFootPosition = this->getLeftFootPosition();
            doComputeMidPoint(_leftFootPosition, target, stepHeight, midPoint);
            break;
        case RIGHT_FOOT:
            _rightFootPosition = this->getRightFootPosition();
            doComputeMidPoint(_rightFootPosition, target, stepHeight, midPoint);
            break;
            
        default:
            break;
    }
}

void StepController::doComputeMidPoint(Eigen::Vector3d currentFootPosition, Eigen::Vector3d target, double stepHeight, Eigen::Vector3d & midPoint) {
    Eigen::Vector3d k; k << 0,0,stepHeight;
    midPoint = currentFootPosition + 0.5*(target - currentFootPosition) + k;
}

Eigen::Vector3d StepController::getLeftFootPosition()
{
    return _model->getSegmentPosition(_model->getSegmentIndex("l_sole")).getTranslation();
}

Eigen::Vector3d StepController::getRightFootPosition()
{
    return _model->getSegmentPosition(_model->getSegmentIndex("r_sole")).getTranslation();
}


void StepController::stop() {
    _rightFoot_TrajThread->stop();
    _leftFoot_TrajThread->stop();
}
