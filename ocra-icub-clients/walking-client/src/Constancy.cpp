#include "walking-client/constraints/Constancy.h"

Constancy::Constancy() : Constraint() {
// Set bounding
    //TODO: This should be input somehow from configuration file
    _S(0) = 0.3;
    _S(1) = 0.3;
}
Constancy::~Constancy() {}

void Constancy::buildMatrixCi() {
    _Ci.resize(8,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero12 = Eigen::VectorXd::Zero(12);
    _Ci << -1,  0,  0,  0, zero12.transpose(),
            1,  0,  0,  0, zero12.transpose(),
            0,  0, -1,  0, zero12.transpose(),
            0,  0,  1,  0, zero12.transpose(),
            0, -1,  0,  0, zero12.transpose(),
            0,  1,  0,  0, zero12.transpose(),
            0,  0,  0, -1, zero12.transpose(),
            0,  0,  0,  1, zero12.transpose();
    OCRA_WARNING("Ci built for Constancy");
}

void Constancy::buildMatrixCii() {
    _Cii.resize(8,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    double sx = _S(0);
    double sy = _S(1);
    _Cii <<  1,  0,  0,  0, -sx,   0,   0,   0, zero8.transpose(),
           -1,  0,  0,  0, -sx,   0,   0,   0, zero8.transpose(),
            0,  0,  1,  0,   0,   0, -sx,   0, zero8.transpose(),
            0,  0, -1,  0,   0,   0, -sx,   0, zero8.transpose(),
            0,  1,  0,  0,   0, -sy,   0,   0, zero8.transpose(),
            0, -1,  0,  0,   0, -sy,   0,   0, zero8.transpose(),
            0,  0,  0,  1,   0,   0,   0, -sy, zero8.transpose(),
            0,  0,  0, -1,   0,   0,   0, -sy, zero8.transpose();
            
    OCRA_WARNING("Cii built for Constancy");
}

void Constancy::buildVectord() {
    _d.resize(_Ci.rows());
    _d.setZero();
    OCRA_WARNING("d built for Constancy");
}
