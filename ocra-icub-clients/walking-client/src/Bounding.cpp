#include "walking-client/constraints/Bounding.h"

Bounding::Bounding() {}
Bounding::~Bounding() {}

void Bounding::buildMatrixCi() {
    Eigen::VectorXd zero12 = Eigen::VectorXd::Zero(12);
    this->_Ci << -1,  0, 1, 0, zero12.transpose(),
                  0, -1, 0, 1, zero12.transpose();
}

void Bounding::buildMatrixCii() {
    this->_Cii = Eigen::MatrixXd::Zero(_Ci.rows(), _Ci.cols());
}
void Bounding::buildVectord() {
    this->_d << 0, 0;
}
