#include "walking-client/constraints/ContactConfigEnforcement.h"

ContactConfigEnforcement::ContactConfigEnforcement() : Constraint() {}

ContactConfigEnforcement::~ContactConfigEnforcement(){}

void ContactConfigEnforcement::buildMatrixCi(){
    _Ci.resize(8,STATE_VECTOR_SIZE);
    Eigen::VectorXd zero7 = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd zero4 = Eigen::VectorXd::Zero(4);
    _Ci << zero4.transpose(), -1, -1,  0,  0,  1, zero7.transpose(),
          zero4.transpose(), -1,  1,  0,  0,  1, zero7.transpose(),
          zero4.transpose(),  0,  0,  1, -1,  1, zero7.transpose(),
          zero4.transpose(),  0,  0, -1,  1,  1, zero7.transpose(),
          zero4.transpose(),  1,  0,  0, -1, -1, zero7.transpose(),
          zero4.transpose(), -1,  0,  0,  1, -1, zero7.transpose(),
          zero4.transpose(),  0, -1,  1,  0, -1, zero7.transpose(),
          zero4.transpose(),  0,  1, -1,  0, -1, zero7.transpose();
    OCRA_INFO("Ci built for ContactConfigEnforcement");
}

void ContactConfigEnforcement::buildMatrixCii(){
    _Cii.resize(_Ci.rows(), _Ci.cols());
    _Cii.setZero();
    OCRA_INFO("Cii built for ContactConfigEnforcement");
}

void ContactConfigEnforcement::buildVectord(){
    _d.resize(_Ci.rows());
    _d << 1, 1, 1, 1, 0, 0, 0, 0;
    OCRA_INFO("d built for ContactConfigEnforcement");
}
