#include "walking-client/constraints/MIQPLinearConstraints.h"

MIQPLinearConstraints::MIQPLinearConstraints(std::shared_ptr<StepController> stepController,
                                             MIQPParameters &miqpParams):
_dt(miqpParams.dt),
_N(miqpParams.N),
_miqpParams(miqpParams),
_stepController(stepController),
_addShapeCtrs(miqpParams.shapeConstraints), 
_addAdmissibilityCtrs(miqpParams.admissibilityConstraints),
_addCoPConstraints(miqpParams.copConstraints),
_addWalkingCtrs(miqpParams.walkingConstraints)
 {
    if (_addShapeCtrs) {
        OCRA_WARNING("This MIQP Controller will add Shape Constraints");
        _shapeCnstr = std::make_shared<ShapeConstraints>();
        _shapeCnstr->init();    
    }
    if (_addAdmissibilityCtrs) {
        OCRA_WARNING("This MIQP Controller will add Admissibility Constraints");
        _admissibilityCnstr = std::make_shared<AdmissibilityConstraints>();
        _admissibilityCnstr->init();
    }
    
    buildMatrixQ();
    buildMatrixT();

    setMatrixAcl();
    setMatrixAcr();
    
    // Initialize number of constraints to 0
    _nConstraints = 0;
    // First build Shape and/or Admissibility Constraints in preview window
    buildShapeAndAdmissibilityInPreviewWindow();
    // Then build CoP constraints in preview window
    _baseOfSupport = std::make_shared<BaseOfSupport>(_stepController,_Q,_T,_miqpParams);
    // Now stack in _A the previous constraints matrices. 
    // If also CoP constraints are added, we need to resize A with the total number of constraints times the size of the input vector \mathcal{X}
     if(_addCoPConstraints) {
         OCRA_WARNING("This MIQP Controller will add CoP Constraints");
         // FIXME: Get the hardcoded 14*miqpParams from Base of Support class. Something like getnConstraints(). Also add to nConstraints those added by baseOfSupport
         Eigen::MatrixXd ACoP(14*_N, _T.cols()*_N); 
         OCRA_INFO("ACoP has size: " << ACoP.rows() << "x" << ACoP.cols());
         _baseOfSupport->getA(ACoP);
         _A.resize(_AShapeAdmiss.rows() + ACoP.rows(), _AShapeAdmiss.cols());
         _A.block(0,0,_AShapeAdmiss.rows(), _AShapeAdmiss.cols()) = _AShapeAdmiss;
         _A.block(_AShapeAdmiss.rows(),0, ACoP.rows(), ACoP.cols()) = ACoP;
         // Increment the number of constraints given by the base of support ones
         // The number of shape and admissibility constraints are added in buildShapeAndAdmissibilityInPreviewWindow()
         _nConstraints += ACoP.rows();
         OCRA_WARNING("This problem will have " << ACoP.rows() << " CoP constraints, " << _AShapeAdmiss.rows() << " shape and admissibility constraints, for a total of: " << _nConstraints << " constraints!");
     } else {
         _A.resize(_AShapeAdmiss.rows(), _AShapeAdmiss.cols());
         _A.block(0,0,_AShapeAdmiss.rows(), _AShapeAdmiss.cols()) = _AShapeAdmiss;
         OCRA_WARNING("Built Matrix A in preview window");
     }
    // Initialize size of rhs
    //TODO: Once I add walking constraints this will change to include the rows added by walking constraints
     if(_addCoPConstraints) {
         Eigen::VectorXd rhsCoP(14*_N);
        _rhs.resize(_fcbarShapeAdmiss.size() + rhsCoP.size());
        OCRA_WARNING("RHS contains: " << _fcbarShapeAdmiss.size() << " shape and admissibility constraints terms and " << rhsCoP.size() << " cop constraints terms, for a total of: " << _rhs.size());
     } else {
         _rhs.resize(_fcbarShapeAdmiss.size());
         OCRA_WARNING("Resized vector rhs");
     }
     
    OCRA_WARNING("MIQPLinearConstraints constructor done");
}

MIQPLinearConstraints::~MIQPLinearConstraints (){}

void MIQPLinearConstraints::setMatrixAcr() {
    if(_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acr.resize(_shapeCnstr->getCii().rows() + _admissibilityCnstr->getCii().rows(), _shapeCnstr->getCii().cols());
        this->_Acr << _shapeCnstr->getCii(), _admissibilityCnstr->getCii();
        OCRA_INFO("Built Acr for Shape and Admissiblity Constraints");
    }
    if(_addShapeCtrs && !_addAdmissibilityCtrs) {
        _Acr.resize(_shapeCnstr->getCii().rows(), _shapeCnstr->getCii().cols());
        this->_Acr << _shapeCnstr->getCii();
        OCRA_INFO("Built Acr for Shape Constraints only");
    }
    if(!_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acr.resize(_admissibilityCnstr->getCii().rows(), _admissibilityCnstr->getCii().cols());
        this->_Acr << _admissibilityCnstr->getCii();
        OCRA_INFO("Built Acr for Admissibility Constraints only");
    }
}

void MIQPLinearConstraints::setMatrixAcl() {
    if(_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acl.resize(_shapeCnstr->getCi().rows() + _admissibilityCnstr->getCi().rows(), _shapeCnstr->getCi().cols());
        this->_Acl << _shapeCnstr->getCi(), _admissibilityCnstr->getCi();
        OCRA_INFO("Built Acl for Shape and Admissiblity Constraints");
    }
    if(_addShapeCtrs && !_addAdmissibilityCtrs) {
        _Acl.resize(_shapeCnstr->getCi().rows(), _shapeCnstr->getCi().cols());
        this->_Acl << _shapeCnstr->getCi();
        OCRA_INFO("Built Acl for Shape Constraints only");
    }
    if(!_addShapeCtrs && _addAdmissibilityCtrs) {
        _Acl.resize(_admissibilityCnstr->getCi().rows(), _admissibilityCnstr->getCi().cols());
        this->_Acl << _admissibilityCnstr->getCi();
        OCRA_INFO("Built Acl for Admissibility Constraints only");
    }
}

void MIQPLinearConstraints::updateRHS(const Eigen::VectorXd& xi_k){
    _rhs.segment(0,_fcbarShapeAdmiss.size()) = _fcbarShapeAdmiss - _BShapeAdmiss * xi_k;
    /** FIXME: TEMPORARY!!  Maybe it's best to have a more generic update method*/
    if (_addCoPConstraints) {
        _baseOfSupport->update(xi_k);
        // TODO: Add to _rhs the base of support terms
        Eigen::VectorXd tmprhs(_rhs.size() - _fcbarShapeAdmiss.size());
        _baseOfSupport->getrhs(tmprhs);
        _rhs.segment(_fcbarShapeAdmiss.size(), tmprhs.size()) = tmprhs;
    }
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
    _Q = Eigen::MatrixXd::Zero(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE);
    _Q.block(10,10,_Ah.rows(),_Ah.cols()) = _Ah;
}

void MIQPLinearConstraints::buildMatrixT() {
    buildBh();
    _T = Eigen::MatrixXd::Identity(STATE_VECTOR_SIZE, INPUT_VECTOR_SIZE);
    _T.block(10,10,6,2) = _Bh;
}

void MIQPLinearConstraints::buildShapeAndAdmissibilityInPreviewWindow(){
    // This builds A in A*X <= fcbar - B*xi_k
    buildAShapeAdmiss();
    // This builds B in A*X <= fcbar - B*xi_k
    buildBShapeAdmiss();
    // This builds fcbar in A*X <= fcbar - B*xi_k
    buildFcBarShapeAdmiss();
    
    // Update the total number of constraints
    _nConstraints += _AShapeAdmiss.rows();
    // Update matrix A
    // TODO: When walking constraints are added, this will be a stack of the two
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
    Eigen::MatrixXd Aexcerpt;
    _AShapeAdmiss.block(0,0,_N*_Acr.rows(), _T.cols()) = AColumn;
    while (j < _N){
        _AShapeAdmiss.block(j*_Acr.rows(), j*_T.cols(), _Acr.rows()*(_N-j), _T.cols()) = AColumn.topRows((_N-j)*_Acr.rows());
        Aexcerpt = AColumn .topRows((_N-j)*_Acr.rows());
        j++;
    }
    Aexcerpt = _AShapeAdmiss.block(0,0,_N*_Acl.rows(), _N*_T.cols());
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
    _fcbarShapeAdmiss.resize(totalRows*_N);
    
    for (unsigned int i=0; i<_N; i++)
        _fcbarShapeAdmiss.segment(i*fc.size(), fc.size()) = fc;
    OCRA_WARNING("Built fcbarShapeAdmiss");
}

unsigned int MIQPLinearConstraints::getTotalNumberOfConstraints() {
    return _nConstraints;
}

void MIQPLinearConstraints::getConstraintsMatrixA(Eigen::MatrixXd &A) {
    if (A.rows() != _A.rows() || A.cols() != _A.cols())
        OCRA_ERROR("Output matrix does not have the right size");
    A = _A;
}
