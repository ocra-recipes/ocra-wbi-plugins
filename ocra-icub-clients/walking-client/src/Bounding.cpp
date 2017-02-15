#include "walking-client/constraints/Bounding.h"

Bounding::Bounding() : Constraint() {}
Bounding::~Bounding() {}

void Bounding::buildMatrixCi() {
    Eigen::VectorXd zero12 = Eigen::VectorXd::Zero(12);
    _Ci.resize(2, SIZE_STATE_VECTOR);
    this->_Ci << -1,  0, 1, 0, zero12.transpose(),
                  0, -1, 0, 1, zero12.transpose();
    OCRA_WARNING("Ci built for Bounding");
}

void Bounding::buildMatrixCii() {
    _Cii.resize(_Ci.rows(), _Ci.cols());
    this->_Cii = Eigen::MatrixXd::Zero(_Ci.rows(), _Ci.cols());
    OCRA_WARNING("Cii built for Bounding");
}
void Bounding::buildVectord() {
    _d.resize(2);
    this->_d << 0, 0;
}
