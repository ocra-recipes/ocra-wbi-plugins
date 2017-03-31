#include "walking-client/BaseOfSupport.h"

BaseOfSupport::BaseOfSupport(std::shared_ptr<StepController> stepController, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& T, MIQPParameters miqpParams):
_stepController(stepController),
_miqpParams(miqpParams),
_Q(Q),
_T(T),
_Ab(Eigen::MatrixXd(4,2)),
_b(Eigen::VectorXd(4)),
_Cp(Eigen::MatrixXd(2, 6)),
_Ci(Eigen::MatrixXd(14,STATE_VECTOR_SIZE)),
_f(Eigen::VectorXd(14))
{
    _A.resize(_Ci.rows()*miqpParams.N, T.cols()*miqpParams.N); _A.setZero();
    _fbar.resize(_f.rows()*miqpParams.N); _fbar.setZero();
    _B.resize(_Ci.rows()*miqpParams.N, _Q.cols()); _B.setZero();
    _rhs.resize(miqpParams.N); _rhs.setZero();
    // Build matrices of the bounding constraints when expressed in the terms of the state vector xi
    buildAb();
    buildCp(_miqpParams.cz, _miqpParams.g);
    buildCi(_Ab, _Cp);
    buildA(_Ci, _Q, _T);
    buildB(_Ci, _Q);
    _f.setZero();
}

BaseOfSupport::~BaseOfSupport(){}
   
void BaseOfSupport::buildAb() {
    _Ab << -1, 0, 1, 0, 0, -1, 0, 1;
    OCRA_WARNING("Built Ab");
}
   
void BaseOfSupport::buildb(const Eigen::Matrix2d& minMaxBoundingBox) {
    _b << -minMaxBoundingBox(0,0) - _miqpParams.marginCoPBounds,
           minMaxBoundingBox(1,0) - _miqpParams.marginCoPBounds,
          -minMaxBoundingBox(0,1) + _miqpParams.marginCoPBounds,
           minMaxBoundingBox(1,1) - _miqpParams.marginCoPBounds;
    OCRA_INFO("Built b");
}
   
void BaseOfSupport::buildCp(double cz, double g) {
    _Cp.setZero();
    _Cp.block(0,0,2,2) = Eigen::MatrixXd::Identity(2, 2);
    _Cp.block(0,4,2,2) = (-cz/g)*Eigen::MatrixXd::Identity(2, 2);
    OCRA_INFO("Built Cp");
}
   
void BaseOfSupport::buildCi(const Eigen::MatrixXd& Ab, const Eigen::MatrixXd& Cp) {
    if (Ab.size() < 1)
        OCRA_ERROR("Reference Ab hasn't been resized");
    if (Cp.size() < 1) 
        OCRA_ERROR("Reference Cp hasn't been resized");
    _Ci.setZero();
    _Ci.block(10,10,4,6) = _Ab*_Cp;
    OCRA_INFO("Built Ci");
}
   
void BaseOfSupport::buildf(){
    _f.segment(10,4) = _b;
}

void BaseOfSupport::buildfbar(const Eigen::VectorXd& f) {
    if (f.size() < 1)
        OCRA_ERROR("f hasn't been resized");
    for (unsigned int i=0; i < _miqpParams.N; i++) {
        _fbar.segment(i*f.size(), f.size()) = f;
    }
}

void BaseOfSupport::buildB(const Eigen::MatrixXd& Ci, const Eigen::MatrixXd& Q){
    for (unsigned int i=0; i < _miqpParams.N; i++) {
        _B.block(i*Ci.rows(), 0, Ci.rows(), _Q.cols()) = Ci*_Q.pow(i+1);
    }
    OCRA_INFO("Built B");
}

void BaseOfSupport::buildA(const Eigen::MatrixXd& Ci, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& T){
    // Create first column of Aeq
    Eigen::MatrixXd AColumn(Ci.rows()*_miqpParams.N, T.cols());
    for (unsigned int i=0; i<_miqpParams.N; i++){
        AColumn.block(i*Ci.rows(), 0, Ci.rows(), T.cols()) = Ci * Q.pow(i) * T;
    }
    // Shift AeqColumn into every column of Aeq to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<_miqpParams.N){
        _A.block(j*Ci.rows(), j*T.cols(), Ci.rows()*(_miqpParams.N-j), T.cols()) = AColumn.topRows((_miqpParams.N-j)*Ci.rows());
        j=j+1;
    }
    OCRA_INFO("Built A");
}

bool BaseOfSupport::update(const Eigen::VectorXd& xi_k) {
    // Get feet corners
    Eigen::MatrixXd feetCorners;
    feetCorners = _stepController->getContact2DCoordinates();
    // Compute the bounding box of the current support configuration
    Eigen::Matrix2d minMaxBoundingBox;
    computeBoundingBox(feetCorners, minMaxBoundingBox);
    // Update right-hand side of constraints
    buildb(minMaxBoundingBox);
    buildf();
    // Matrices in the preview window s.t.
    // A*X <= fbar - B*xi_k
    // Therefore we can prebuild A, fbar and B which are not time-dependent
    buildfbar(_f);
    _rhs = _fbar - _B*xi_k;
    return true;
}

void BaseOfSupport::computeBoundingBox(const Eigen::MatrixXd &feetCorners, Eigen::Matrix2d &minMaxBoundingBox){
    if (minMaxBoundingBox.size() < 1)
        OCRA_ERROR("Output matrix hasn't been resized");
    
    _poly.clear();
    OCRA_INFO("Feet corners given by StepController are: ");
    std::cout << feetCorners << std::endl;
    for (unsigned int i=0 ; i<feetCorners.rows(); i++) {
        _poly.outer().push_back(point(feetCorners(i,0), feetCorners(i,1)));
    }
    boost::geometry::correct(_poly);
    
    boost::geometry::envelope(_poly, _bbox);

//     using boost::geometry::dsv;
//     std::cout
//     << "polygon: " << dsv(_poly) << std::endl
    
    minMaxBoundingBox(0,0) = boost::geometry::get<boost::geometry::min_corner, 0>(_bbox);
    minMaxBoundingBox(0,1) = boost::geometry::get<boost::geometry::min_corner, 1>(_bbox);
    minMaxBoundingBox(1,0) = boost::geometry::get<boost::geometry::max_corner, 0>(_bbox);
    minMaxBoundingBox(1,1) = boost::geometry::get<boost::geometry::max_corner, 1>(_bbox);

    OCRA_WARNING("Points in bounding box");
    std::cout << "min: " << minMaxBoundingBox(0,0) << ", " << minMaxBoundingBox(0,1) << std::endl;
    std::cout << "max: " << minMaxBoundingBox(1,0) << ", " << minMaxBoundingBox(1,1) << std::endl;
    
}

void BaseOfSupport::getA(Eigen::MatrixXd &output) {
    if (output.rows() != _A.rows() || output.cols() != _A.cols())
        OCRA_ERROR("Output matrix hasn't been resized");

    if (output.rows() !=  _A.rows() || output.cols() != _A.cols())
        OCRA_ERROR("Malformed constraint matrix container A. It should have size: " << _A.rows() << "x" << _A.cols());
    
    output = _A;
    OCRA_INFO("Built A");
}

void BaseOfSupport::getrhs(Eigen::VectorXd &output) {
    if (output.size() != _rhs.size()) {
        OCRA_ERROR("Malformed constraint vector container RHS. It should have size: " << _rhs.size());
    }
    output = _rhs;
}
