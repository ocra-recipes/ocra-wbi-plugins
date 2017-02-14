#include "walking-client/constraints/Constancy.h"

Constancy::Constancy() {
// Set bounding
    //TODO: This should be input somehow from configuration file
    _S(0) = 0.01;
    _S(1) = 0.01;
}
Constancy::~Constancy() {}

void Constancy::buildMatrixCi() {
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    double sx = _S(0);
    double sy = _S(1);
    _Ci <<  1,  0,  0,  0, -sx,   0,   0,   0, zero8.transpose(),
           -1,  0,  0,  0, -sx,   0,   0,   0, zero8.transpose(),
            0,  0,  1,  0,   0,   0, -sx,   0, zero8.transpose(),
            0,  0, -1,  0,   0,   0, -sx,   0, zero8.transpose(),
            0,  1,  0,  0,   0, -sy,   0,   0, zero8.transpose(),
            0, -1,  0,  0,   0, -sy,   0,   0, zero8.transpose(),
            0,  0,  0,  1,   0,   0,   0, -sy, zero8.transpose(),
            0,  0,  0, -1,   0,   0,   0, -sy, zero8.transpose();
}

void Constancy::buildMatrixCii() {
    Eigen::VectorXd zero12 = Eigen::VectorXd::Zero(12);
    _Cii << -1,  0,  0,  0, zero12.transpose(),
            1,  0,  0,  0, zero12.transpose(),
            0,  0, -1,  0, zero12.transpose(),
            0,  0,  1,  0, zero12.transpose(),
            0, -1,  0,  0, zero12.transpose(),
            0,  1,  0,  0, zero12.transpose(),
            0,  0,  0, -1, zero12.transpose(),
            0,  0,  0,  1, zero12.transpose();
}

void Constancy::buildVectord() {
    _d = Eigen::VectorXd::Zero(_Ci.rows());
}
