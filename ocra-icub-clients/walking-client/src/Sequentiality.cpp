#include "walking-client/constraints/Sequentiality.h"

Sequentiality::Sequentiality() : Constraint() {}
Sequentiality::~Sequentiality() {}

void Sequentiality::buildMatrixCi() {
    _Ci.resize(2,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    _Ci << zero4.transpose(), 1, 0, 1, 0, zero8.transpose(),
           zero4.transpose(), 0, 1, 0, 1, zero8.transpose();
    OCRA_WARNING("Ci built for sequentiality");
}

void Sequentiality::buildMatrixCii() {
    _Cii.resize(1, SIZE_STATE_VECTOR);
    _Cii = Eigen::MatrixXd::Zero(_Ci.rows(), _Ci.cols());
    OCRA_WARNING("Cii built for sequentiality");
}

void Sequentiality::buildVectord() {
    _d.resize(2);
    _d << 1, 1;
    OCRA_WARNING("d built for sequentiality");
}

