#include "walking-client/constraints/MIQPLinearConstraints.h"

MIQPLinearConstraints::MIQPLinearConstraints(unsigned int dt, 
                                             unsigned int N, 
                                             std::shared_ptr<StepController> stepController,
                                             bool addShapeCtrs, 
                                             bool addAdmissibilityCtrs,
                                             bool addCoPConstraints,
                                             bool addWalkingCtrs):
_dt(dt), 
_N(N), 
_stepController(stepController),
_addShapeCtrs(addShapeCtrs), 
_addAdmissibilityCtrs(addAdmissibilityCtrs),
_addCoPConstraints(addCoPConstraints),
_addWalkingCtrs(addWalkingCtrs)
 {
    if (_addShapeCtrs) {
        _shapeCnstr = std::make_shared<ShapeConstraints>();
        _shapeCnstr->init();    
    }
    if (_addAdmissibilityCtrs) {
        _admissibilityCnstr = std::make_shared<AdmissibilityConstraints>();
        _admissibilityCnstr->init();
    }
    
    buildMatrixQ();
    buildMatrixT();

    setMatrixAcl();
    setMatrixAcr();
    
    buildShapeAndAdmissibilityInPreviewWindow();
    
    
    
    // Initialize size of _rhs
    //TODO: Once I add walking constraints this will change to include the rows added by walking constraints
    _rhs.resize(_fcbarShapeAdmiss.size());
    OCRA_WARNING("MIQPLinearConstraints constructor done");
}

MIQPLinearConstraints::~MIQPLinearConstraints (){}

void MIQPLinearConstraints::setMatrixAcr() {
    if(_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acr.resize(_shapeCnstr->getCii().rows() + _admissibilityCnstr->getCii().rows(), _shapeCnstr->getCii().cols());
        this->_Acr << _shapeCnstr->getCii(), _admissibilityCnstr->getCii();
        OCRA_WARNING("Built Acr for Shape and Admissiblity Constraints");
    }
    if(_addShapeCtrs && !_addAdmissibilityCtrs) {
        _Acr.resize(_shapeCnstr->getCii().rows(), _shapeCnstr->getCii().cols());
        this->_Acr << _shapeCnstr->getCii();
        OCRA_WARNING("Built Acr for Shape Constraints only");
    }
    if(!_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acr.resize(_admissibilityCnstr->getCii().rows(), _admissibilityCnstr->getCii().cols());
        this->_Acr << _admissibilityCnstr->getCii();
        OCRA_WARNING("Built Acr for Admissibility Constraints only");
    }
}

void MIQPLinearConstraints::setMatrixAcl() {
    if(_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acl.resize(_shapeCnstr->getCi().rows() + _admissibilityCnstr->getCi().rows(), _shapeCnstr->getCi().cols());
        this->_Acl << _shapeCnstr->getCi(), _admissibilityCnstr->getCi();
        OCRA_WARNING("Built Acl for Shape and Admissiblity Constraints");
    }
    if(_addShapeCtrs && !_addAdmissibilityCtrs) {
        _Acl.resize(_shapeCnstr->getCi().rows(), _shapeCnstr->getCi().cols());
        this->_Acl << _shapeCnstr->getCi();
        OCRA_WARNING("Built Acl for Shape Constraints only");
    }
    if(!_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acl.resize(_admissibilityCnstr->getCi().rows(), _admissibilityCnstr->getCi().cols());
        this->_Acl << _admissibilityCnstr->getCi();
        OCRA_WARNING("Built Acl for Admissibility Constraints only");
    }
}

void MIQPLinearConstraints::updateRHS(Eigen::VectorXd xi_k){
    _rhs = _fcbarShapeAdmiss - _BShapeAdmiss * xi_k;
    OCRA_WARNING("Updated RHS");
}

void MIQPLinearConstraints::getRHS(Eigen::VectorXd &rhs) {
    rhs = _rhs;
}

void MIQPLinearConstraints::buildBh() {
    _Bh.resize(6,2);
    double dt = (double) _dt/1000;
    _Bh << (pow(dt,3)/6)*Eigen::Matrix2d::Identity(), (pow(dt,2)/2)*Eigen::Matrix2d::Identity(), dt*Eigen::Matrix2d::Identity();
}

void MIQPLinearConstraints::buildAh() {
    _Ah.resize(6,6);
    _Ah.setIdentity();
    double dt = (double) _dt/1000;
    _Ah.block(0,2,2,2) = dt*Eigen::Matrix2d::Identity();
    _Ah.block(0,4,2,2) = (pow(dt,2)/2)*Eigen::Matrix2d::Identity();
    _Ah.block(2,4,2,2) = dt*Eigen::Matrix2d::Identity();
}

void MIQPLinearConstraints::buildMatrixQ() {
    buildAh();
    _Q = Eigen::MatrixXd::Identity(SIZE_STATE_VECTOR, SIZE_STATE_VECTOR);
    _Q.block(10,10,_Ah.rows(),_Ah.cols()) = _Ah;
}

void MIQPLinearConstraints::buildMatrixT() {
    buildBh();
    _T = Eigen::MatrixXd::Identity(SIZE_STATE_VECTOR, SIZE_INPUT_VECTOR);
    _T.block(10,10,6,2) = _Bh;
}

void MIQPLinearConstraints::buildShapeAndAdmissibilityInPreviewWindow(){
    // This builds A in A*X <= fc - B*xi_k
    buildAShapeAdmiss();
    // This builds B in A*X <= fc - B*xi_k
    buildBShapeAdmiss();
    // This builds fc in A*X <= fc - B*xi_k
    buildFcBarShapeAdmiss();
    
    // Update the total number of constraints
    _nConstraints = _AShapeAdmiss.rows();
    // Update matrix A
    // TODO: When walking constraints are added, this will be a stack of the two
    _A = _AShapeAdmiss;
}

void MIQPLinearConstraints::buildAShapeAdmiss() {
    _AShapeAdmiss.resize(_Acr.rows()*_N, _T.cols()*_N);
    _AShapeAdmiss.setZero();
    // Create first column
    Eigen::MatrixXd AColumn(_Acr.rows()*_N, _T.cols());
    AColumn.block(0, 0, _Acr.rows(), _T.cols()) = _Acr*_T;
    for (unsigned int i=1; i<=_N-1; i++){
        AColumn.block(i*_Acr.rows(), 0, _Acr.rows(), _T.cols()) = _Acl*_Q.pow(i-1)*_T + _Acr*_Q.pow(i)*_T;
    }
    // Shift AColumn to take the form of a lower diagonal toeplitz matrix
    unsigned int j=1;
    while (j < _N){
        _AShapeAdmiss.block(j*_Acr.rows(), j*_T.cols(), _Acr.rows()*(_N-j), _T.cols()) = AColumn.topRows((_N-j)*_Acr.rows());
        j++;
    }
    OCRA_WARNING("Built AShapeAdmiss");
}

void MIQPLinearConstraints::buildBShapeAdmiss() {
    OCRA_INFO("_Acr size is: " << _Acr.rows() << " cols: " << _Acr.cols()  << " _Q size is: " << _Q.rows() << " " << _Q.cols());
    _BShapeAdmiss.resize(_Acr.rows()*_N, _Q.cols());
    _BShapeAdmiss.setZero();

    for (unsigned int i = 1; i <= _N; i++) {
        _BShapeAdmiss.block((i-1)*_Acr.rows(), 0, _Acr.rows(), _Q.cols()) = _Acl*_Q.pow(i-1) + _Acr*_Q.pow(i);
    }
    OCRA_WARNING("Built BShapeAdmiss");
}

void MIQPLinearConstraints::buildFcBarShapeAdmiss() {
    unsigned int totalRows;
    Eigen::VectorXd fc;
    if (_shapeCnstr && _addAdmissibilityCtrs) {
        totalRows = _shapeCnstr->getd().size() + _admissibilityCnstr->getd().size();
        fc.resize(totalRows);
        fc << _shapeCnstr->getd(), _admissibilityCnstr->getd();
    } 
    if (_shapeCnstr && !_admissibilityCnstr) {
        totalRows = _shapeCnstr->getd().size();
        fc.resize(totalRows);
        fc << _shapeCnstr->getd();
    }
    if (!_shapeCnstr && _admissibilityCnstr) {
        totalRows = _admissibilityCnstr->getd().size();
        fc.resize(totalRows);
        fc << _admissibilityCnstr->getd();
    }
    OCRA_WARNING("Size of fc: " << totalRows);
    _fcbarShapeAdmiss.resize(totalRows*_N);
    
    OCRA_WARNING("fc: " << fc.transpose());
    for (unsigned int i=0; i<_N; i++)
        _fcbarShapeAdmiss.segment(i*fc.size(), fc.size()) = fc;
    OCRA_WARNING("Built fcbarShapeAdmiss");
}

unsigned int MIQPLinearConstraints::getTotalNumberOfConstraints() {
    return _nConstraints;
}

void MIQPLinearConstraints::getConstraintsMatrixA(Eigen::MatrixXd &A) {
    A = _A;
}
