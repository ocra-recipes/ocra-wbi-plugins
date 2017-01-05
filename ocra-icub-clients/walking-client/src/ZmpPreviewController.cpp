#include "walking-client/ZmpPreviewController.h"
#include "unsupported/Eigen/MatrixFunctions"


ZmpPreviewController::ZmpPreviewController(const double period, std::shared_ptr<ZmpPreviewParams> parameters) :
cz(parameters->cz),
dt(period/1000),
Nc(parameters->Nc),
nu(parameters->nu),
nw(parameters->nw),
nb(parameters->nb),
Nu(buildNu(nu, Nc)),
Nw(buildNw(nw, Nc)),
Nb(buildNb(nb, Nc)),
Ah(buildAh(dt)),
Bh(buildBh(dt)),
Cp(buildCp(cz,g)),
Gp(buildGp(Cp,Ah,Nc)),
Hp(buildHp(Cp,Bh,Ah,Nc)),
Ch(buildCh()),
Gh(buildGh(Ch,Ah,Nc)),
Hh(buildHh(Ch,Bh,Ah,Nc))
{ 
    OCRA_INFO("Parameters passed to ZmpPreviewController: \n cz: " << parameters->cz << " Nc: " << parameters->Nc << " nu " << parameters->nu << " nw: " << parameters->nw << " nb: " << parameters->nb);
}

ZmpPreviewController::~ZmpPreviewController() {
}

bool ZmpPreviewController::initialize() {
    return true;
}

bool ZmpPreviewController::computeOptimalInput(Eigen::VectorXd zmpRef, Eigen::VectorXd comVelRef, Eigen::VectorXd hk, Eigen::VectorXd &optimalU) {
    Eigen::MatrixXd A = Hp.transpose()*Nb*Hp + Nu + Hh.transpose()*Nw*Hh;
    Eigen::MatrixXd b = Hp.transpose()*Nb*(zmpRef - Gp*hk) + Hh.transpose()*Nw*(comVelRef - Gh*hk);
    optimalU = A.colPivHouseholderQr().solve(b);
    return true;
}

void ZmpPreviewController::integrateComJerk(Eigen::VectorXd comJerk, Eigen::VectorXd hk, Eigen::VectorXd &hkk) {
    hkk = Ah*hk + Bh*comJerk;
}

void ZmpPreviewController::tableCartModel(Eigen::Vector2d hk, Eigen::VectorXd ddhk, Eigen::Vector2d& p) {   
    // Another expression is Ch*hkk
    p = hk - this->cz/this->g * ddhk;
}
    
// ************ Output preview matrices and COM state space matrices ******************

Eigen::MatrixXd ZmpPreviewController::buildAh(const double dt) {
    Eigen::MatrixXd Ah(6,6);
    Ah.setIdentity();
    Ah.block(0,2,2,2) = dt*Eigen::Matrix2d::Identity();
    Ah.block(0,4,2,2) = (pow(dt,2)/2)*Eigen::Matrix2d::Identity();
    Ah.block(2,4,2,2) = dt*Eigen::Matrix2d::Identity();
    return Ah;
}

Eigen::MatrixXd ZmpPreviewController::buildBh(const double dt){
    Eigen::MatrixXd Bh(6,2);
    Bh << (pow(dt,3)/6)*Eigen::Matrix2d::Identity(), (pow(dt,2)/2)*Eigen::Matrix2d::Identity(), dt*Eigen::Matrix2d::Identity();
    return Bh;
}

Eigen::MatrixXd ZmpPreviewController::buildCp(const double cz, const double g) {
    Eigen::MatrixXd Cp(2,6);
    Cp << Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero(), (-cz/g)*Eigen::Matrix2d::Identity();
    OCRA_INFO("About to compute Cp with cz: " << cz << "and g: " << g);
    OCRA_INFO("Cp: \n" << Cp);
    return Cp;
}

Eigen::MatrixXd ZmpPreviewController::buildGp(Eigen::MatrixXd Cp, Eigen::MatrixXd Ah, const int Nc) {
    const int CpRows = Cp.rows();
    const int AhCols = Ah.cols();
    Eigen::MatrixXd Gp(Cp.rows()*Nc, Ah.cols());
    Gp.setZero();
    for (unsigned int i=0; i<Nc; i++) {
        Gp.block(i*CpRows, 0, CpRows, AhCols) = Cp*Ah.pow(i+1);
    }
    OCRA_INFO("Gp: \n" << Gp);
    return Gp;
}

Eigen::MatrixXd ZmpPreviewController::buildHp(Eigen::MatrixXd Cp, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc) {
    const int CpRows = Cp.rows();
    const int BhCols = Bh.cols();
    Eigen::MatrixXd Hp(CpRows*Nc, BhCols*Nc);
    Hp.setZero();
    // Create first column
    Eigen::MatrixXd HpColumn(CpRows*Nc, BhCols);
    for (unsigned int i=0; i<Nc; i++){
        HpColumn.block(i*CpRows, 0, CpRows, BhCols) = Cp*Ah.pow(i)*Bh;
    }
    // Shift HpColumn into every column of Hp to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<Nc){
        Hp.block(j*CpRows, j*BhCols, CpRows*(Nc-j), BhCols) = HpColumn.topRows((Nc-j)*CpRows);
        j=j+1;
    }
    OCRA_INFO("Hp: \n" << Hp);
    return Hp;
}

Eigen::MatrixXd ZmpPreviewController::buildCh() {
    Eigen::MatrixXd Ch(2,6);
    Ch << Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero();
    return Ch;
}

Eigen::MatrixXd ZmpPreviewController::buildGh(Eigen::MatrixXd Ch, Eigen::MatrixXd Ah, const int Nc) {
    const int ChRows = Ch.rows();
    const int AhCols = Ah.cols();
    Eigen::MatrixXd Gh(ChRows*Nc, AhCols);
    Gh.setZero();
    for (unsigned int i=0; i<Nc; i++) {
        Gh.block(i*ChRows, 0, ChRows, AhCols) = Ch*Ah.pow(i+1);
    }
    return Gh;
}

Eigen::MatrixXd ZmpPreviewController::buildHh(Eigen::MatrixXd Ch, Eigen::MatrixXd Bh, Eigen::MatrixXd Ah, const int Nc) {
    const int ChRows = Ch.rows();
    const int BhCols = Bh.cols();
    Eigen::MatrixXd Hh(ChRows*Nc, BhCols*Nc);
    Hh.setZero();
    // Create first column
    Eigen::MatrixXd HhColumn(ChRows*Nc, BhCols);
    for (unsigned int i=0; i<Nc; i++){
        HhColumn.block(i*ChRows, 0, ChRows, BhCols) = Ch*Ah.pow(i)*Bh;
    }
    // Shift HhColumn into every column of Hh to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<Nc){
        Hh.block(j*ChRows, j*BhCols, ChRows*(Nc-j), BhCols) = HhColumn.topRows((Nc-j)*ChRows);
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

Eigen::MatrixXd ZmpPreviewController::buildNw(const double nw, const int Nc) {
    Eigen::MatrixXd Nw;
    Nw.setIdentity(2*Nc, 2*Nc);
    Nw = nw*Nw;
    return Nw;
}

Eigen::MatrixXd ZmpPreviewController::buildNb(const double nb, const int Nc) {
    Eigen::MatrixXd Nb;
    Nb.setIdentity(2*Nc, 2*Nc);
    Nb = nb*Nb;
    return Nb;
}
