#include "walking-client/constraints/Sequentiality.h"

Sequentiality::Sequentiality() {}
Sequentiality::~Sequentiality() {}

void Sequentiality::buildMatrixCi() {
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    _Ci << zero4, 1, 0, 1, 0, zero8.transpose(),
           zero4, 0, 1, 0, 1, zero8.transpose();
}

void Sequentiality::buildMatrixCii() {
    _Cii = Eigen::MatrixXd::Zero(_Ci.rows(), _Ci.cols());
}

void Sequentiality::buildVectord() {
    _d << 1, 1;
}

