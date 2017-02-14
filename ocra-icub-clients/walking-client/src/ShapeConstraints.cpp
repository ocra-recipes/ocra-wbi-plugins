#include "walking-client/constraints/ShapeConstraints.h"

ShapeConstraints::ShapeConstraints() {
}
ShapeConstraints::~ShapeConstraints() {}

void ShapeConstraints::buildMatrixCi() {
    unsigned int totalRows = _bounding.getCi().rows() + _constancy.getCi().rows() + _sequentiality.getCi().rows();
    unsigned int totalCols = SIZE_STATE_VECTOR;
    _Ci.resize(totalRows, totalCols);
    _Ci << _bounding.getCi(), _constancy.getCi(), _sequentiality.getCi();
}
void ShapeConstraints::buildMatrixCii() {
    unsigned int totalRows = _Ci.rows();
    unsigned int totalCols = _Cii.cols();
    _Cii.resize(totalRows, totalCols);
    _Cii << _bounding.getCii(), _constancy.getCii(), _sequentiality.getCii();
}
void ShapeConstraints::buildVectord() {
    unsigned int dSize = _bounding.getd().size() + _constancy.getd().size() + _sequentiality.getd().size();
    _d.resize(dSize);
    _d << _bounding.getd(), _constancy.getd(), _sequentiality.getd();
}
