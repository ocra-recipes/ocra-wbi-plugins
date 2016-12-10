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
    
    // Compute ZMP for right foot
    Eigen::Vector2d rightFootZmp; rightFootZmp.setZero();
    Eigen::VectorXd rightWrenchInWorld(6); rightWrenchInWorld.setZero();
    computeFootZMP(RIGHT_FOOT, rawRightFootWrench, rightFootZmp, rightWrenchInWorld);
    
    Eigen::Vector2d fVecz;
    fVecz << rightWrenchInWorld(2), leftWrenchInWorld(2);
    double divisor = 1/(fVecz(0) + fVecz(1));
    globalZMP = divisor*(Eigen::MatrixXd(2,2) << rightFootZmp, leftFootZmp).finished() * fVecz;
    

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
    Eigen::Matrix2d kfVec = Eigen::Matrix2d::Identity();
    kfVec(0,0) = _params->kfx;
    kfVec(1,1) = _params->kfy;
    Eigen::Vector2d error;
    error = pd - p;
    dhd = _params->m * (_params->g/_params->cz) * kfVec * error;
    
    static Eigen::Vector2d previousError = Eigen::Vector2d::Zero();
    Eigen::Matrix2d kdVec = Eigen::Matrix2d::Identity();
    kdVec(0,0) = _params->kdx;
    kdVec(1,1) = _params->kdy;
    //TODO: Remove this constant dt
    Eigen::Vector2d derivative = (1/_params->controllerPeriod) * kdVec * (error - previousError);
    previousError = error;
    dhd += derivative;
    
    return true;
}

void ZmpController::computehdd(Eigen::Vector3d comPosition, Eigen::Vector2d refZMP, Eigen::Vector2d &ddh) {
    ddh = (_params->g/_params->cz)*(comPosition.topRows(2) - refZMP);
}

void ZmpController::computeh(Eigen::Vector2d prevComPosition, Eigen::Vector2d prevComVel, Eigen::Vector2d &intComPosition) {
    intComPosition = prevComPosition + _params->controllerPeriod*prevComVel;
}

void ZmpController::getLeftFootPosition(Eigen::Vector3d &leftFootPosition) {
    leftFootPosition = _model->getSegmentPosition(_model->getSegmentIndex("l_sole")).getTranslation();
}

void ZmpController::getRightFootPosition(Eigen::Vector3d &rightFootPosition) {
    rightFootPosition = _model->getSegmentPosition(_model->getSegmentIndex("r_sole")).getTranslation();
}
