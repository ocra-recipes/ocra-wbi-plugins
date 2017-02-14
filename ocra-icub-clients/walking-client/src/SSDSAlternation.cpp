#include "walking-client/constraints/SSDSAlternation.h"

SSDSAlternation::SSDSAlternation(){}

SSDSAlternation::~SSDSAlternation(){}

void SSDSAlternation::buildMatrixCi() {
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Ci << zero4.transpose(), 1, 0,  1, 0, 0,  1, zero6,
                                                                            zero4.transpose(),  1, 0,  1, 0, 0, -1, zero6,
                                                                            zero4.transpose(), -1, 0, -1, 0, 0,  1, zero6,
                                                                            zero4.transpose(), -1, 0, -1, 0, 0, -1, zero6;
}

void SSDSAlternation::buildMatrixCii(){
    Eigen::VectorXd zero9 = Eigen::VectorXd::Zero(9);
    _Cii << zero9.transpose(),  1, 0,
                                                                            zero9.transpose(),  -1, 0,
                                                                            zero9.transpose(),  -1, 0,
                                                                            zero9.transpose(),   1, 0;
}

void SSDSAlternation::buildVectord(){
    _d << 2, 0, 0, 0;
}
