#include "walking-client/constraints/ContactConfigHistory.h"

ContactConfigHistory::ContactConfigHistory() : Constraint(){}

ContactConfigHistory::~ContactConfigHistory(){}

void ContactConfigHistory::buildMatrixCi(){
    _Ci.resize(2,STATE_VECTOR_SIZE);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Ci << zero8.transpose(), -1, 1, zero6.transpose(),
           zero8.transpose(),  1, 1, zero6.transpose();

    OCRA_INFO("Ci built for ContactConfigHistory");
}

void ContactConfigHistory::buildMatrixCii(){
    _Cii.resize(2, STATE_VECTOR_SIZE);
    Eigen::VectorXd zero8 = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd zero7 = Eigen::VectorXd::Zero(7);
    _Cii << zero8.transpose(),  1, zero7.transpose(),
            zero8.transpose(), -1, zero7.transpose();
    OCRA_INFO("Cii built for ContactConfigHistory");
}

void ContactConfigHistory::buildVectord(){
    _d.resize(_Ci.rows());
    _d << 1, 1;
    OCRA_INFO("d built for ContactConfigHistory");
}
