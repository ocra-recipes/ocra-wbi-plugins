#include "walking-client/ZmpController.h"

ZmpController::ZmpController(const int period,
                             std::shared_ptr<ocra::Model> modelPtr,
                             std::shared_ptr<ZmpControllerParams> parameters) :
_params(parameters),
_model(modelPtr)
{
    
}

ZmpController::~ZmpController() {
    
}

bool ZmpController::computeFootZMP(FOOT whichFoot,
                                   Eigen::VectorXd wrench,
                                   Eigen::Vector2d &footZMP,
                                   Eigen::VectorXd &wrenchInWorldRef,
                                   const double tolerance) {
    Eigen::MatrixXd adjointTransposed;
    switch (whichFoot) {
        case LEFT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed);
            break;
        case RIGHT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed);
            break;
        default:
            break;
    }
    
    wrenchInWorldRef = adjointTransposed * wrench;
    
    // If abs(f_z) < tolerance
    if (fabs(wrenchInWorldRef(2)) < tolerance) {
        footZMP.setZero();
        return true;
    }
    
    // Build matrix A stablishing the linear relationship between the foot wrench and its zmp
    Eigen::MatrixXd A(2,6);
    A.setZero();
    const double fz = wrenchInWorldRef(2);
    A(0,0) = -_params->d/fz;
    A(1,1) = -_params->d/fz;
    A(0,4) = -1;
    A(1,3) = 1;
    
    footZMP = A*wrenchInWorldRef;
    return true;
}

bool ZmpController::computeGlobalZMPFromSensors(Eigen::VectorXd rawLeftFootWrench,
                                                Eigen::VectorXd rawRightFootWrench,
                                                Eigen::Vector2d &globalZMP) {
    // Compute ZMP for left foot
    Eigen::Vector2d leftFootZmp; leftFootZmp.setZero();
    Eigen::VectorXd leftWrenchInWorld(6); leftWrenchInWorld.setZero();
    computeFootZMP(LEFT_FOOT, rawLeftFootWrench, leftFootZmp, leftWrenchInWorld);
    // Compute ZMP for right foot
    Eigen::Vector2d rightFootZmp; rightFootZmp.setZero();
    Eigen::VectorXd rightWrenchInWorld(6); rightWrenchInWorld.setZero();
    computeFootZMP(RIGHT_FOOT, rawRightFootWrench, rightFootZmp, rightWrenchInWorld);
    
    Eigen::Vector2d fVecz;
    fVecz << rightWrenchInWorld(2), leftWrenchInWorld(2);
    const double divisor = 1/(fVecz(0) + fVecz(1));
    globalZMP = divisor*(Eigen::MatrixXd(2,2) << rightFootZmp, leftFootZmp).finished() * fVecz;
    
    return true;
}

void ZmpController::getFTSensorAdjointMatrix(FOOT whichFoot, Eigen::MatrixXd &T) {
    std::string prefixFoot;
    switch (whichFoot) {
        case LEFT_FOOT:
            prefixFoot = "l_";
            break;
        case RIGHT_FOOT:
            prefixFoot = "r_";
            break;
        default:
            break;
    }
    
    Eigen::Displacementd sensorPoseInWorld = _model->getSegmentPosition(std::string(prefixFoot + "foot_ft_sensor"));
    T = sensorPoseInWorld.adjoint().transpose();
    
}

bool ZmpController::computehd(Eigen::Vector2d p, Eigen::Vector2d pd, Eigen::Vector2d &dhd){
    dhd = _params->kf * _params->m * (_params->cz/_params->g) * ( p - pd );
    return true;
}

void ZmpController::getLeftFootPosition(Eigen::Vector3d &leftFootPosition) {
    leftFootPosition = _model->getSegmentPosition(_model->getSegmentIndex("l_sole")).getTranslation();
}

void ZmpController::getRightFootPosition(Eigen::Vector3d &rightFootPosition) {
    rightFootPosition = _model->getSegmentPosition(_model->getSegmentIndex("r_sole")).getTranslation();
}




