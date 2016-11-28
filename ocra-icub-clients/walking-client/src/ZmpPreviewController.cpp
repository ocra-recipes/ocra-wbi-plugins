#include "walking-client/ZmpPreviewController.h"
#include "unsupported/Eigen/MatrixFunctions"


ZmpPreviewController::ZmpPreviewController(const int period, ZmpPreviewParams parameters) :
cz(parameters.cz),
dt(period/1000),
Nc(parameters.Nc),
nu(parameters.nu),
nw(parameters.nw),
nb(parameters.nb),
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
{ }

ZmpPreviewController::~ZmpPreviewController(){
}

bool ZmpPreviewController::initialize(){
    return true;
}

bool ZmpPreviewController::initialize(std::string fileOfReferences){
    return true;
}

bool ZmpPreviewController::computeOptimalInput(){
    return true;
}

// ************ Output preview matrices and COM state space matrices ******************

Eigen::MatrixXd ZmpPreviewController::buildAh(const double dt){
    Eigen::MatrixXd Ah(3,6);
    Ah.row(0) << 1, 1, dt, dt, pow(dt,2)/2, pow(dt,2)/2;
    Ah.row(1) << 0, 0, 1, 1, dt, dt;
    Ah.row(2) << 0, 0, 0, 0, 1, 1;
    return Ah;
}

Eigen::MatrixXd ZmpPreviewController::buildBh(const double dt){
    Eigen::MatrixXd Bh(1,3);
    Bh(0) = pow(dt,3)/6;
    Bh(1) = pow(dt,2)/2;
    Bh(2) = dt;
    return Bh;
}

Eigen::MatrixXd ZmpPreviewController::buildCp(const double cz, const double g) {
    Eigen::MatrixXd Cp(1,6);
    Cp << 1, 1, 0, 0, -cz/g, -cz/g;
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
    return Hp;
}

Eigen::MatrixXd ZmpPreviewController::buildCh() {
    Eigen::MatrixXd Ch(1,6);
    Ch << Eigen::RowVector2d::Constant(0), Eigen::RowVector2d::Constant(1), Eigen::RowVector2d::Constant(0);
    return Ch;
}

Eigen::MatrixXd ZmpPreviewController::buildGh(Eigen::MatrixXd Ch, Eigen::MatrixXd Ah, const int Nc) {
    const int ChRows = Ch.rows();
    const int AhCols = Ah.cols();
    Eigen::MatrixXd Gh(Ch.rows()*Nc, Ah.cols());
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
    Nu.setIdentity(Nc, Nc);
    Nu = nu*Nu;
    return Nu;
}

Eigen::MatrixXd ZmpPreviewController::buildNw(const double nw, const int Nc) {
    Eigen::MatrixXd Nw;
    Nw.setIdentity(Nc, Nc);
    Nw = nw*Nw;
    return Nw;
}

Eigen::MatrixXd ZmpPreviewController::buildNb(const double nb, const int Nc) {
    Eigen::MatrixXd Nb;
    Nb.setIdentity(Nc, Nc);
    Nb = nb*Nb;
    return Nb;
}
