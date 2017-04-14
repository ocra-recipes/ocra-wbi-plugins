#include "walking-client/ZmpPreviewController.h"
#include "unsupported/Eigen/MatrixFunctions"


ZmpPreviewController::ZmpPreviewController(const double period, std::shared_ptr<ZmpPreviewParams> parameters) :
cz(parameters->cz),
dt(period/1000),
Np(parameters->Np),
Nc(parameters->Nc),
nu(parameters->nu),
nw(parameters->nw),
nb(parameters->nb),
Nu(buildNu(nu, Nc)),
Nw(buildNw(nw, Np)),
Nb(buildNb(nb, Np)),
Ah(buildAh(dt)),
Bh(buildBh(dt)),
Cp(buildCp(cz,g)),
Gp(buildGp(Cp,Ah,Np)),
Hp(buildHp(Cp,Bh,Ah,Nc,Np)),
Ch(buildCh()),
Gh(buildGh(Ch,Ah,Np)),
Hh(buildHh(Ch,Bh,Ah,Nc,Np))
{ 
    AOptimal = Eigen::MatrixXd(2*Nc,2*Nc).setZero();
//  Wieber's expression    
//     AOptimal = Hp.transpose() * Hp + Nu;
    AOptimal = Hp.transpose()*Nb*Hp + Nu + Hh.transpose()*Nw*Hh;
    bOptimal = Eigen::MatrixXd(2*Nc,1).setZero();
    OCRA_INFO("Parameters passed to ZmpPreviewController: \n cz: " << parameters->cz << " Nc: " << parameters->Nc << " nu " << parameters->nu << " nw: " << parameters->nw << " nb: " << parameters->nb);
    OCRA_ERROR("After constructor, Ah: \n" << Ah);
    OCRA_ERROR("After constructor, Bh: \n" << Bh);
}

ZmpPreviewController::~ZmpPreviewController() {
}

bool ZmpPreviewController::initialize() {
    return true;
}

void ZmpPreviewController::integrateCom(Eigen::VectorXd comJerk, Eigen::VectorXd hk, Eigen::VectorXd &hkk) {
    hkk = Ah*hk + Bh*comJerk;
}

void ZmpPreviewController::tableCartModel(Eigen::Vector2d hk, Eigen::VectorXd ddhk, Eigen::Vector2d& p) {   
    p = hk - this->cz/this->g * ddhk;
}

bool ZmpPreviewController::computeFootZMP(FOOT whichFoot,
                                   Eigen::VectorXd wrench,
                                   Eigen::Vector2d &footZMP,
                                   Eigen::VectorXd &wrenchInWorldRef,
                                   ocra::Model::Ptr model,
                                   const double tolerance) {
    Eigen::MatrixXd adjointTransposed;
    Eigen::Vector3d sensorPosition;
    switch (whichFoot) {
        case LEFT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed, sensorPosition, model);
            break;
        case RIGHT_FOOT:
            getFTSensorAdjointMatrix(whichFoot, adjointTransposed, sensorPosition, model);
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

bool ZmpPreviewController::computeGlobalZMPFromSensors(Eigen::VectorXd rawLeftFootWrench,
                                                Eigen::VectorXd rawRightFootWrench,
                                                ocra::Model::Ptr model,
                                                Eigen::Vector2d &globalZMP) {
    // Compute ZMP for left foot
    Eigen::Vector2d leftFootZmp; leftFootZmp.setZero();
    Eigen::VectorXd leftWrenchInWorld(6); leftWrenchInWorld.setZero();
    computeFootZMP(LEFT_FOOT, rawLeftFootWrench, leftFootZmp, leftWrenchInWorld, model);
    
    // Compute ZMP for right foot
    Eigen::Vector2d rightFootZmp; rightFootZmp.setZero();
    Eigen::VectorXd rightWrenchInWorld(6); rightWrenchInWorld.setZero();
    computeFootZMP(RIGHT_FOOT, rawRightFootWrench, rightFootZmp, rightWrenchInWorld, model);
    
    Eigen::Vector2d fVecz;
    fVecz << rightWrenchInWorld(2), leftWrenchInWorld(2);
    double divisor = 1/(fVecz(0) + fVecz(1));
    globalZMP = divisor*(Eigen::MatrixXd(2,2) << rightFootZmp, leftFootZmp).finished() * fVecz;
    

    return true;
}

void ZmpPreviewController::getFTSensorAdjointMatrix(FOOT whichFoot, Eigen::MatrixXd &T, Eigen::Vector3d &sensorPosition, ocra::Model::Ptr model) {
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
    
    Eigen::Displacementd sensorPoseInWorld = model->getSegmentPosition(std::string(prefixFoot + "foot"));
//    std::cout << prefixFoot + "Foot sensor is at: " << std::endl << sensorPoseInWorld.getTranslation().transpose() << std::endl;
    sensorPosition = sensorPoseInWorld.getTranslation();
    T = sensorPoseInWorld.adjoint().transpose();
    
}

void ZmpPreviewController::tableCartModel(Eigen::VectorXd hkk, Eigen::Vector2d& p) {   
    p = this->Cp * hkk;
}
    
// ************ Output preview matrices and COM state space matrices ******************

Eigen::MatrixXd ZmpPreviewController::buildAh(const double dt) {
    Eigen::MatrixXd Ah(6,6);
    Ah.setIdentity();
    Ah.block(0,2,2,2) = dt*Eigen::Matrix2d::Identity();
    Ah.block(0,4,2,2) = (pow(dt,2)/2)*Eigen::Matrix2d::Identity();
    Ah.block(2,4,2,2) = dt*Eigen::Matrix2d::Identity();
    OCRA_ERROR("Ah is: " << Ah);
    return Ah;
}

Eigen::MatrixXd ZmpPreviewController::buildBh(const double dt){
    Eigen::MatrixXd Bh(6,2);
    Bh << (pow(dt,3)/6)*Eigen::Matrix2d::Identity(), (pow(dt,2)/2)*Eigen::Matrix2d::Identity(), dt*Eigen::Matrix2d::Identity();
    OCRA_ERROR("Bh is: " << Bh);
    return Bh;
}

Eigen::MatrixXd ZmpPreviewController::buildCp(const double cz, const double g) {
    Eigen::MatrixXd Cp(2,6);
    Cp << Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero(), (-cz/g)*Eigen::Matrix2d::Identity();
//     OCRA_INFO("About to compute Cp with cz: " << cz << "and g: " << g);
//     OCRA_INFO("Cp: \n" << Cp);
    return Cp;
}

Eigen::MatrixXd ZmpPreviewController::buildGp(Eigen::MatrixXd Cp, Eigen::MatrixXd Ah, const int Np) {
    const int CpRows = Cp.rows();
    const int AhCols = Ah.cols();
    Eigen::MatrixXd Gp(Cp.rows()*Np, Ah.cols());
    Gp.setZero();
    for (unsigned int i=0; i<Np; i++) {
        Gp.block(i*CpRows, 0, CpRows, AhCols) = Cp*Ah.pow(i+1);
    }
//     OCRA_INFO("Gp: \n" << Gp);
    return Gp;
}

Eigen::MatrixXd ZmpPreviewController::buildHp(Eigen::MatrixXd Cp, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc, const int Np) {
    const int CpRows = Cp.rows();
    const int BhCols = Bh.cols();
    Eigen::MatrixXd Hp(CpRows*Np, BhCols*Nc);
    Hp.setZero();
    // Create first column
    Eigen::MatrixXd HpColumn(CpRows*Np, BhCols);
    for (unsigned int i=0; i<Np; i++){
        HpColumn.block(i*CpRows, 0, CpRows, BhCols) = Cp*Ah.pow(i)*Bh;
    }
    // Shift HpColumn into every column of Hp to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<Nc){
        Hp.block(j*CpRows, j*BhCols, CpRows*(Np-j), BhCols) = HpColumn.topRows((Np-j)*CpRows);
        j=j+1;
    }
//     OCRA_INFO("Hp: \n" << Hp);
    return Hp;
}

Eigen::MatrixXd ZmpPreviewController::buildCh() {
    Eigen::MatrixXd Ch(2,6);
    Ch << Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero();
    return Ch;
}

Eigen::MatrixXd ZmpPreviewController::buildGh(Eigen::MatrixXd Ch, Eigen::MatrixXd Ah, const int Np) {
    const int ChRows = Ch.rows();
    const int AhCols = Ah.cols();
    Eigen::MatrixXd Gh(ChRows*Np, AhCols);
    Gh.setZero();
    for (unsigned int i=0; i<Np; i++) {
        Gh.block(i*ChRows, 0, ChRows, AhCols) = Ch*Ah.pow(i+1);
    }
    return Gh;
}

Eigen::MatrixXd ZmpPreviewController::buildHh(Eigen::MatrixXd Ch, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc, const int Np) {
    const int ChRows = Ch.rows();
    const int BhCols = Bh.cols();
    Eigen::MatrixXd Hh(ChRows*Np, BhCols*Nc);
    Hh.setZero();
    // Create first column
    Eigen::MatrixXd HhColumn(ChRows*Np, BhCols);
    //TODO: I think this should be Np - 1
    for (unsigned int i=0; i<Np; i++){
        HhColumn.block(i*ChRows, 0, ChRows, BhCols) = Ch*Ah.pow(i)*Bh;
    }
    // Shift HhColumn into every column of Hh to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<Nc){
        Hh.block(j*ChRows, j*BhCols, ChRows*(Np-j), BhCols) = HhColumn.topRows((Np-j)*ChRows);
        j=j+1;
    }
    return Hh;
}

Eigen::MatrixXd ZmpPreviewController::buildNu(const double nu, const int Nc) {
    Eigen::MatrixXd Nu;
    Nu.setIdentity(2*Nc, 2*Nc);
    Nu = nu*Nu;
    return Nu;
}

Eigen::MatrixXd ZmpPreviewController::buildNw(const double nw, const int Np) {
    Eigen::MatrixXd Nw;
    Nw.setIdentity(2*Np, 2*Np);
    Nw = nw*Nw;
    return Nw;
}

Eigen::MatrixXd ZmpPreviewController::buildNb(const double nb, const int Np) {
    Eigen::MatrixXd Nb;
    Nb.setIdentity(2*Np, 2*Np);
    Nb = nb*Nb;
    return Nb;
}
