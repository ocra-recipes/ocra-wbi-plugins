#include "walking-client/constraints/ContactConfigHistory.h"

ContactConfigHistory::ContactConfigHistory(){}

ContactConfigHistory::~ContactConfigHistory(){}

void ContactConfigHistory::buildMatrixCi(){
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    _Ci << zero8.transpose(),  1, 0, 0, 0,
           zero8.transpose(), -1, 0, 0, 0;
}

void ContactConfigHistory::buildMatrixCii(){
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    _Cii << zero8.transpose(), -1, 1, 0, 0,
            zero8.transpose(), 1, 1, 0, 0;
}

void ContactConfigHistory::buildVectord(){
    _d << 1, 1;
}
