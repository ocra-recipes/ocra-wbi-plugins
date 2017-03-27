#include "walking-client/constraints/Sequentiality.h"

Sequentiality::Sequentiality() : Constraint() {}
Sequentiality::~Sequentiality() {}

void Sequentiality::buildMatrixCi() {
    _Ci.resize(2,STATE_VECTOR_SIZE);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    _Ci << zero4.transpose(), 1, 0, 1, 0, zero8.transpose(),
           zero4.transpose(), 0, 1, 0, 1, zero8.transpose();
    OCRA_INFO("Ci built for sequentiality");
}

void Sequentiality::buildMatrixCii() {
    _Cii.resize(_Ci.rows(), _Ci.cols());
    _Cii.setZero();
    OCRA_INFO("Cii built for sequentiality");
}

void Sequentiality::buildVectord() {
    _d.resize(_Ci.rows());
    _d << 1, 1;
    OCRA_INFO("d built for sequentiality");
}

