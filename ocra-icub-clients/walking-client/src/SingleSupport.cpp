#include "walking-client/constraints/SingleSupport.h"

SingleSupport::SingleSupport(){}

SingleSupport::~SingleSupport(){}

void SingleSupport::buildMatrixCi(){
    //TODO: temporarily hardcoding this bound S
    double sx = 0.01;
    _S(0) = sx;
    double sy = 0.01;
    _S(1) = 0.01;
    _Ci << 1,  0,  -1, 0,  0, 0, 0,  -sx, 0, 0,
                                                          -1,  0,  1,  0,  0, 0, 0, -sx, 0, 0,
                                                           0,  1,  0, -1,  0, 0, 0, -sy, 0, 0,
                                                           0, -1,  0,  1,  0, 0, 0, -sy, 0, 0;
}
void SingleSupport::buildMatrixCii(){
    _Ci = Eigen::MatrixXd::Zero(_Ci.rows(), _Ci.cols());
}

void SingleSupport::buildVectord(){
    _d << 0, 0, 0, 0;
}
