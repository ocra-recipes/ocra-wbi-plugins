#include "walking-client/constraints/AdmissibilityConstraints.h"

AdmissibilityConstraints::AdmissibilityConstraints() {}

AdmissibilityConstraints::~AdmissibilityConstraints() {}

void AdmissibilityConstraints::buildMatrixCi() {
    unsigned int totalRows = _ssdsAlternation.getCi().rows() + _singleSupport.getCi().rows() + _contactConfigHistory.getCi().rows() + _contactConfigEnforcement.getCi().rows();
    unsigned int totalCols = SIZE_STATE_VECTOR;
    _Ci.resize(totalRows,totalCols);
    _Ci << _ssdsAlternation.getCi(), _singleSupport.getCi(), _contactConfigHistory.getCi(), _contactConfigEnforcement.getCi();
}

void AdmissibilityConstraints::buildMatrixCii() {
    unsigned int totalRows = _Ci.rows();
    unsigned int totalCols = SIZE_STATE_VECTOR;
    _Cii.resize(totalRows,totalCols);
    _Cii << _ssdsAlternation.getCii(), _singleSupport.getCii(), _contactConfigHistory.getCii(), _contactConfigEnforcement.getCii();
}

void AdmissibilityConstraints::buildVectord() {
    _d << _ssdsAlternation.getd(), _singleSupport.getd(), _contactConfigHistory.getd(), _contactConfigEnforcement.getd();
}

