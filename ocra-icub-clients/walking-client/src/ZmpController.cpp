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
    Eigen::Vector3d sensorPosition;
    switch (whichFoot) {
        case LEFT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed, sensorPosition);
            break;
        case RIGHT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed, sensorPosition);
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
    A << ((-sensorPosition(2))*Eigen::Matrix2d::Identity()), sensorPosition.topRows(2), (Eigen::Matrix2d() << 0, -1, 1, 0).finished(), Eigen::Vector2d::Zero();
    std::cout << "Matrix A" << std::endl << A << std::endl;
    double fz = wrenchInWorldRef(2);
    
    footZMP = (1/fz)*A*wrenchInWorldRef;
    return true;
}

bool ZmpController::computeGlobalZMPFromSensors(Eigen::VectorXd rawLeftFootWrench,
                                                Eigen::VectorXd rawRightFootWrench,
                                                Eigen::Vector2d &globalZMP) {
    // Compute ZMP for left foot
    Eigen::Vector2d leftFootZmp; leftFootZmp.setZero();
    Eigen::VectorXd leftWrenchInWorld(6); leftWrenchInWorld.setZero();
    computeFootZMP(LEFT_FOOT, rawLeftFootWrench, leftFootZmp, leftWrenchInWorld);
    std::cout << "Left foot zmp: " << std::endl << leftFootZmp.transpose() << std::endl;
    // Compute ZMP for right foot
    Eigen::Vector2d rightFootZmp; rightFootZmp.setZero();
    Eigen::VectorXd rightWrenchInWorld(6); rightWrenchInWorld.setZero();
    computeFootZMP(RIGHT_FOOT, rawRightFootWrench, rightFootZmp, rightWrenchInWorld);
    std::cout << "Right foot zmp: " << std::endl << rightFootZmp.transpose() << std::endl;
    
//    std::cout << "Right foot wrench in world: " << rightWrenchInWorld << std::endl;
//    std::cout << "Left foot wrench in world: " << leftWrenchInWorld << std::endl;
    
    Eigen::Vector2d fVecz;
    fVecz << rightWrenchInWorld(2), leftWrenchInWorld(2);
    double divisor = 1/(fVecz(0) + fVecz(1));
    globalZMP = divisor*(Eigen::MatrixXd(2,2) << rightFootZmp, leftFootZmp).finished() * fVecz;
    
    std::cout << "Global zmp: " << std::endl << globalZMP.transpose() << std::endl;
    
    return true;
}

void ZmpController::getFTSensorAdjointMatrix(FOOT whichFoot, Eigen::MatrixXd &T, Eigen::Vector3d &sensorPosition) {
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
    
    Eigen::Displacementd sensorPoseInWorld = _model->getSegmentPosition(std::string(prefixFoot + "foot"));
//    std::cout << prefixFoot + "Foot sensor is at: " << std::endl << sensorPoseInWorld.getTranslation().transpose() << std::endl;
    sensorPosition = sensorPoseInWorld.getTranslation();
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




