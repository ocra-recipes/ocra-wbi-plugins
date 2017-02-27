#include "walking-client/constraints/SingleSupport.h"

SingleSupport::SingleSupport() : Constraint(){}

SingleSupport::~SingleSupport(){}

void SingleSupport::buildMatrixCi(){
    Eigen::VectorXd zero5 = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
    _Ci.resize(4,SIZE_STATE_VECTOR);
    //TODO: temporarily hardcoding this bound S
    double sx = 0.01;
    _S(0) = sx;
    double sy = 0.01;
    _S(1) = sy;
    _Ci << 1,  0,  -1, 0, zero5.transpose(),  -sx, zero6.transpose(),
          -1,  0,  1,  0, zero5.transpose(), -sx, zero6.transpose(),
           0,  1,  0, -1, zero5.transpose(), -sy, zero6.transpose(),
           0, -1,  0,  1, zero5.transpose(), -sy, zero6.transpose();
    OCRA_WARNING("Ci build for SingleSupport");
}
void SingleSupport::buildMatrixCii(){
    _Cii.resize(_Ci.rows(), _Ci.cols());
    _Cii.setZero();
    OCRA_WARNING("Cii build for SingleSupport");
}

void SingleSupport::buildVectord(){
    _d.resize(_Ci.rows());
    _d << 0, 0, 0, 0;
    OCRA_WARNING("d build for SingleSupport");
}
