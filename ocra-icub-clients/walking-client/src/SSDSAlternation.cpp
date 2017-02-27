#include "walking-client/constraints/SSDSAlternation.h"

SSDSAlternation::SSDSAlternation() : Constraint(){}

SSDSAlternation::~SSDSAlternation(){}

void SSDSAlternation::buildMatrixCi() {
    _Ci.resize(4,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero9 = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Ci << zero9.transpose(),  1, zero6.transpose(),
           zero9.transpose(),  -1, zero6.transpose(),
           zero9.transpose(),  -1, zero6.transpose(),
           zero9.transpose(),   1, zero6.transpose();
    OCRA_WARNING("Ci built for SSDSAlternation");
}

void SSDSAlternation::buildMatrixCii(){
    _Cii.resize(4,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Cii << zero4.transpose(), 1, 0,  1, 0, 0,  1, zero6.transpose(),
            zero4.transpose(), 1, 0,  1, 0, 0, -1, zero6.transpose(),
            zero4.transpose(),-1, 0, -1, 0, 0,  1, zero6.transpose(),
            zero4.transpose(),-1, 0, -1, 0, 0, -1, zero6.transpose();
    OCRA_WARNING("Cii built for SSDSAlternation");
}

void SSDSAlternation::buildVectord(){
    _d.resize(_Ci.rows());
    _d << 2, 0, 0, 0;
    OCRA_WARNING("d built for SSDSAlternation");
}
