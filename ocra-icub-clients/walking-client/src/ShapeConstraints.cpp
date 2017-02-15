#include "walking-client/constraints/ShapeConstraints.h"

ShapeConstraints::ShapeConstraints() : Constraint() {
    _bounding.init();
    _constancy.init();
    _sequentiality.init();
}
ShapeConstraints::~ShapeConstraints() {}

void ShapeConstraints::buildMatrixCi() {
    OCRA_WARNING("Building MatrixCi for Shape Constraints");
    unsigned int totalRows = _bounding.getCi().rows() + _constancy.getCi().rows() + _sequentiality.getCi().rows();
    unsigned int totalCols = SIZE_STATE_VECTOR;
    _Ci.resize(totalRows, totalCols);
    _Ci << _bounding.getCi(), _constancy.getCi(), _sequentiality.getCi();
    OCRA_WARNING("Built Ci for Shape Constraints");
}
void ShapeConstraints::buildMatrixCii() {
    unsigned int totalRows = _Ci.rows();
    unsigned int totalCols = _Ci.cols();
    _Cii.resize(totalRows, totalCols);
    _Cii << _bounding.getCii(), _constancy.getCii(), _sequentiality.getCii();
    OCRA_WARNING("Built Cii for Shape Constraints");
}
void ShapeConstraints::buildVectord() {
    unsigned int dSize = _bounding.getd().size() + _constancy.getd().size() + _sequentiality.getd().size();
    _d.resize(dSize);
    _d << _bounding.getd(), _constancy.getd(), _sequentiality.getd();
    OCRA_WARNING("Built d for Shape Constraints");
}
