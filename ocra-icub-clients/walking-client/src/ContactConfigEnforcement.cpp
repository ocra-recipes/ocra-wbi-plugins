#include "walking-client/constraints/ContactConfigEnforcement.h"

ContactConfigEnforcement::ContactConfigEnforcement(){}

ContactConfigEnforcement::~ContactConfigEnforcement(){}

void ContactConfigEnforcement::buildMatrixCi(){
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
}

void ContactConfigEnforcement::buildMatrixCii(){
    _Cii = Eigen::MatrixXd::Zero(_Ci.rows(), _Cii.cols());
}

void ContactConfigEnforcement::buildVectord(){
    _d << 1, 1, 1, 1, 0, 0, 0, 0;
}
