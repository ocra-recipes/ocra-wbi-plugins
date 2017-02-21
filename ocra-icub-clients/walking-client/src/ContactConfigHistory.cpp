#include "walking-client/constraints/ContactConfigHistory.h"

ContactConfigHistory::ContactConfigHistory() : Constraint(){}

ContactConfigHistory::~ContactConfigHistory(){}

void ContactConfigHistory::buildMatrixCi(){
    _Ci.resize(2,SIZE_STATE_VECTOR);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero7 = Eigen::VectorXd::Zero(7);
    _Ci << zero8.transpose(),  1, zero7.transpose(),
           zero8.transpose(), -1, zero7.transpose();
    OCRA_WARNING("Ci built for ContactConfigHistory");
}

void ContactConfigHistory::buildMatrixCii(){
    _Cii.resize(2, SIZE_STATE_VECTOR);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Cii << zero8.transpose(), -1, 1, zero6.transpose(),
            zero8.transpose(), 1, 1, zero6.transpose();
    OCRA_WARNING("Cii built for ContactConfigHistory");
}

void ContactConfigHistory::buildVectord(){
    _d.resize(_Ci.rows());
    _d << 1, 1;
    OCRA_WARNING("d built for ContactConfigHistory");
}
