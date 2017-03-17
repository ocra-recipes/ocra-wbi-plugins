#include "walking-client/MIQPController.h"

using namespace MIQP;

MIQPController::MIQPController(MIQPParameters params, ocra::Model::Ptr robotModel, std::shared_ptr<StepController> stepController, const Eigen::MatrixXd &comStateRef) : RateThread(params.dt),
_robotModel(robotModel),
_stepController(stepController),
_miqpParams(params),
_comStateRef(comStateRef),
_period(params.dt),
_lb(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_ub(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_xi_k(Eigen::VectorXd(STATE_VECTOR_SIZE)),
_X_kn(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_Ah(Eigen::MatrixXd(6,6)),
_Bh(Eigen::MatrixXd(6,2)),
_Q(Eigen::MatrixXd(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE)),
_T(STATE_VECTOR_SIZE, INPUT_VECTOR_SIZE),
_C_H(6,STATE_VECTOR_SIZE),
_C_P(2, STATE_VECTOR_SIZE),
_C_B(2,STATE_VECTOR_SIZE),
_P_H(_miqpParams.N * _C_H.rows(), _Q.cols()),
_P_P(_miqpParams.N * _C_P.rows(), _Q.cols()),
_P_B(_miqpParams.N * _C_B.rows(), _Q.cols()),
_R_H(_C_H.rows()*_miqpParams.N, _T.cols()*_miqpParams.N),
_R_P(_C_P.rows()*_miqpParams.N, _T.cols()*_miqpParams.N),
_R_B(_C_B.rows()*_miqpParams.N, _T.cols()*_miqpParams.N),
_Sw(_R_H.rows(), _R_H.rows()),
_H_N_r(6*_miqpParams.N)

{
    buildAh(_period, _Ah);
    buildBh(_period, _Bh);
    buildQ(_Q);
    buildT(_T);
    buildC_H(_C_H);
    buildC_P(_C_P);
    buildC_B(_C_B);
    buildPreviewStateMatrix(_C_H, _P_H);
    buildPreviewStateMatrix(_C_P, _P_P);
    buildPreviewStateMatrix(_C_B, _P_B);
    buildPreviewInputMatrix(_C_H, _R_H);
    buildPreviewInputMatrix(_C_P, _R_P);
    buildPreviewInputMatrix(_C_B, _R_B);
    buildSw(_Sw);
    buildNb(_Nb);
    buildNx(_Nx);
    buildH_N(_H_N);

//     OCRA_WARNING("Built Ah");
//     std::cout << _Ah << std::endl;
// 
//     OCRA_WARNING("Built Bh");
//     std::cout << _Bh << std::endl;
// 
//     OCRA_WARNING("Built Q");
//     std::cout << _Q<< std::endl;
// 
//     OCRA_WARNING("Built T");
//     std::cout << _T<< std::endl;
// 
//     OCRA_WARNING("Built _C_H");
//     std::cout << _C_H << std::endl;
// 
//     OCRA_WARNING("Built _C_P");
//     std::cout << _C_P << std::endl;
// 
//     OCRA_WARNING("Built _C_B");
//     std::cout << _C_B << std::endl;
// 
//     OCRA_WARNING("Built _P_H");
//     std::cout << _P_H << std::endl;
// 
//     OCRA_WARNING("Built _P_P");
//     std::cout << _P_P << std::endl;
// 
//     OCRA_WARNING("Built _P_B");
//     std::cout << _P_B << std::endl;
// 
//     OCRA_WARNING("Built _R_H");
//     std::cout << _R_H << std::endl;
// 
//     OCRA_WARNING("Built _R_P");
//     std::cout << _R_P << std::endl;
// 
//     OCRA_WARNING("Built _R_B");
//     std::cout << _R_B << std::endl;
// 
//     OCRA_WARNING("Built _Sw");
//     std::cout << _Sw << std::endl;
// 
//     OCRA_WARNING("Built _Nb");
//     std::cout << _Nb << std::endl;
// 
//     OCRA_WARNING("Built _Nx");
//     std::cout << _Nx << std::endl;

    _k = 0;
}

MIQPController::~MIQPController() {
//    OCRA_WARNING("Built an MIQP Controller object");
}

bool MIQPController::threadInit() {

    // Instantiate MIQP state object
    _state = std::make_shared<MIQPState>(_robotModel);
    updateStateVector();

    // Set lower and upper bounds
    setLowerAndUpperBounds();

    // Instantiate MIQPLinearConstraints object and update constraints matrix _Aineq
    // FIXME: For now TESTING only with shape, admissibility and cop constraints!! Don't forget to add walking constraints.
    _constraints = std::make_shared<MIQPLinearConstraints>(_stepController, _miqpParams, true, true, true, false);
    _Aineq.resize(_constraints->getTotalNumberOfConstraints(),  INPUT_VECTOR_SIZE * _miqpParams.N );
    _constraints->getConstraintsMatrixA(_Aineq);

    // Resize _Bineq. However since it's state-dependent, it will be updated in the run() method
    _Bineq.resize(_Aineq.rows());

    std::cout << "Writing Aineq: of size: " << _Aineq.rows() << " x " << _Aineq.cols() << std::endl;

    // Resize _Aeq and _Beq
    _Aeq.resize(_miqpParams.N, INPUT_VECTOR_SIZE*_miqpParams.N);
    _Beq.resize(_miqpParams.N);
    buildEqualityConstraintsMatrices(_xi_k, _Aeq, _Beq);

    // Setup eigen-gurobi object with 12*N variables, N equality constraints and rows-of-Aineq inequality constraints.
    try {
        OCRA_INFO("About to build eigen-gurobi problem");
        _eigGurobi.problem(INPUT_VECTOR_SIZE*_miqpParams.N, _Aeq.rows(), _Aineq.rows());

        // In the previous initialization all variables are assumed continuous by default.
        setBinaryVariables();
    }
    catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    OCRA_WARNING("Finished MIQPController initialization");
    return true;
}

void MIQPController::threadRelease() {
//    delete _env;
}

void MIQPController::run() {
    // Update state vector
    updateStateVector();

    // Update constraints.
    // NOTE: _Aineq is time-invariant and thus built only once, while _Bineq is state dependant
    // (also depends on a history of states when walking constraints are included).
    _constraints->updateRHS(_xi_k);
    _constraints->getRHS(_Bineq);
//    OCRA_WARNING("RHS Retrieved");

    // Updates RHS of equality constraints. For now, contains only Simultaneity
    updateEqualityConstraints(_xi_k, _Beq);

    setCOMStateRefInPreviewWindow(_k, _H_N_r);
    setLinearPartObjectiveFunction();

    try {
    // TODO: Watch out! _eigGurobi will add a 1/2. Therefore the 2. Check that this is correct.
    // FIXME: The expression for _H_N only includes jerk regularization terms
    _eigGurobi.solve(2*_H_N, _linearTermTransObjFunc, _Aeq, _Beq, _Aineq, _Bineq, _lb, _ub);
//     _eigGurobi.inform();

    // Get the solution
    _X_kn = _eigGurobi.result();
    OCRA_WARNING("Optimal is: ")
    std::cout << _X_kn.topRows(INPUT_VECTOR_SIZE).transpose() << std::endl;
    } catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    // Write solution to file for plots
    std::string home = std::string(_miqpParams.home + "MIQP/");
    writeToFile(0.100*_k, _X_kn.topRows(INPUT_VECTOR_SIZE), home);
    _k++;
}

void MIQPController::writeToFile(const double& time, const Eigen::VectorXd& X_kn, std::string& home) {
    Eigen::VectorXd tmp(INPUT_VECTOR_SIZE+1);
    tmp << time, X_kn;
    ocra::utils::writeInFile(tmp, std::string(home + "solution.txt"), true);
}

void MIQPController::setBinaryVariables()
{
    int m = 0;
    while( m < INPUT_VECTOR_SIZE*_miqpParams.N) {
        // Set binary variables (4->9) i.e. alpha_x, alpha_y, beta_x, beta_y, delta, gamma
        for (int i = 4; i <= 9; i++) {
            _eigGurobi.setVariableType(m+i, GRB_BINARY);
        }
        m += INPUT_VECTOR_SIZE;
    }
}

void MIQPController::setCOMStateRefInPreviewWindow(unsigned int k, Eigen::VectorXd &H_N_r) {
    unsigned int j = 0;
    // FIXME: Pass an actual reference of CoM states
    for (unsigned int i = k + 1; i <= k + _miqpParams.N; i++) {
        H_N_r.segment(j, 6) = _comStateRef.row(i);
        j += 6;
    }
}

void MIQPController::setLowerAndUpperBounds() {
    // N blocks of [a_0 a_1 b_0 b_1 alpha_0 alpha_1 beta_0 beta_1 delta gamma u_x u_y]
    _lb.setZero();
    _ub.setOnes();
    unsigned int k = 0;
    while (k < INPUT_VECTOR_SIZE*_miqpParams.N) {
        for (unsigned int j = 0; j < 4; j++)
            _lb[k+j] = -100;
        for (unsigned int j = 10; j < INPUT_VECTOR_SIZE; j++)
            _lb[k+j] = -100;
        k += INPUT_VECTOR_SIZE;
    }
    k = 0;
    while (k < INPUT_VECTOR_SIZE*_miqpParams.N) {
        for (unsigned int j = 0; j < 4; j++)
            _ub[k+j] = 100;
        for (unsigned int j = 10; j < INPUT_VECTOR_SIZE; j++)
            _ub[k+j] = 100;
        k += INPUT_VECTOR_SIZE;
    }
}

void MIQPController::updateStateVector() {
    _state->updateStateVector();
    _state->getFullState(_xi_k);
    OCRA_INFO("State: \n" << *_state);
}

void MIQPController::setLinearPartObjectiveFunction() {
     Eigen::VectorXd a = -2*(_H_N_r - _P_H*_xi_k).transpose()*_Sw*_R_H;
     Eigen::VectorXd b = 2*((_P_P - _P_B)*_xi_k).transpose();
     Eigen::VectorXd c = b*_Nb;
     Eigen::MatrixXd d = (_R_P - _R_B);
//    OCRA_WARNING("Set Linear Part of the Obj Function");
    _linearTermTransObjFunc = a + (c*d);
}

void MIQPController::buildAh(int dt, Eigen::MatrixXd &Ah) {
    Ah.setIdentity();
    double dt_ = (double) dt/1000;
    Ah.block(0,2,2,2) = dt_*Eigen::Matrix2d::Identity();
    Ah.block(0,4,2,2) = (pow(dt_,2)/2)*Eigen::Matrix2d::Identity();
    Ah.block(2,4,2,2) = dt_*Eigen::Matrix2d::Identity();
}

void MIQPController::buildBh(int dt, Eigen::MatrixXd &Bh){
    double dt_ = (double)dt/1000;
    Bh << (pow(dt_,3)/6)*Eigen::Matrix2d::Identity(), (pow(dt_,2)/2)*Eigen::Matrix2d::Identity(), dt_*Eigen::Matrix2d::Identity();
    std::cout << "Bh is: \n" << std::endl;
    std::cout << Bh << std::endl;
}

void MIQPController::buildQ(Eigen::MatrixXd &Q) {
    Q.setZero();
    _Q.block(10,10,_Ah.rows(),_Ah.cols()) = _Ah;
    OCRA_WARNING("Built Q");
    std::cout << Q << std::endl;
}

void MIQPController::buildT(Eigen::MatrixXd &T) {
    T.setZero();
    T.block(0,0,10,10) = Eigen::MatrixXd::Identity(10, 10);
    T.block(10,10,6,2) = _Bh;
    OCRA_WARNING("Built T");
    std::cout << T << std::endl;
}

void MIQPController::buildC_H(Eigen::MatrixXd &C_H) {
    C_H.setZero();
    C_H.block(0,10,6,6) = Eigen::MatrixXd::Identity(6,6);
//    OCRA_WARNING("Built C_H");
}

void MIQPController::buildC_P(Eigen::MatrixXd &C_P) {
    C_P.setZero();
    C_P.block(0,10,2,2) = Eigen::MatrixXd::Identity(2, 2);
    C_P.block(0,14,2,2) = (-_miqpParams.cz/_miqpParams.g)*Eigen::MatrixXd::Identity(2, 2);
//    OCRA_WARNING("Built C_P");
}

void MIQPController::buildC_B(Eigen::MatrixXd &C_B) {
    C_B.setZero();
    C_B.block(0,0,2,2) = 0.5*Eigen::Matrix2d::Identity();
    C_B.block(0,2,2,2) = 0.5*Eigen::Matrix2d::Identity();
//    OCRA_WARNING("Built C_B");
}

void MIQPController::buildH_N(Eigen::MatrixXd &H_N) {
    H_N = Eigen::MatrixXd(_miqpParams.N*INPUT_VECTOR_SIZE, _miqpParams.N*INPUT_VECTOR_SIZE);
//     OCRA_INFO("[ " << _R_H.cols() << " x " << _R_H.rows() << " x " << " " << _Sw.rows() << " x " << _Sw.cols())
    H_N =   _R_H.transpose()*_Sw*_R_H + (_R_P - _R_B).transpose() * _Nb * (_R_P - _R_B) + _Nx;
//    OCRA_WARNING("Built H_N");
}

void MIQPController::buildNb(Eigen::MatrixXd &Nb) {
    Nb = Eigen::MatrixXd::Identity(2*_miqpParams.N, 2*_miqpParams.N);
//    OCRA_WARNING("Built Nb");
}

void MIQPController::buildNx(Eigen::MatrixXd &Nx) {
    Eigen::VectorXd vecToRepeat(12);
    // FIXME: Hardcoding regularization on ALL variables. This should be only for the jerk
    // For now this is ok since I haven't added the walking constraints, thus helping
    // the problem to be feasible.
    vecToRepeat << (Eigen::VectorXd(10) << Eigen::VectorXd::Constant(10,1)).finished(), 1, 1;
    // replicate over the preview window
    Eigen::VectorXd diagonal = vecToRepeat.replicate(1,_miqpParams.N);
    // Transform into diagonal matrix
    Nx = diagonal.asDiagonal();
}

void MIQPController::buildSw(Eigen::MatrixXd &Sw) {
    // FIXME: For the time being this is hardcoded to use the com velocity references only
    // Create the diagonal as a vector
    OCRA_INFO("Initial size of Sw: " << Sw.rows() << ", " << Sw.cols());
    Eigen::VectorXd vecToRepeat(6); vecToRepeat << 0, 0, 1, 1, 0, 0;
    // Replicate
    Eigen::VectorXd diagonal = vecToRepeat.replicate(1,_miqpParams.N);
    // Transform into diagonal matrix
    Sw = diagonal.asDiagonal();
//     Sw = Eigen::MatrixXd::Identity(6*_miqpParams.N, 6*_miqpParams.N);
//    OCRA_WARNING("Built Sw");
}


void MIQPController::buildPreviewStateMatrix(const Eigen::MatrixXd &C, Eigen::MatrixXd &P) {
    P.setZero();
    for (unsigned int i=0; i<_miqpParams.N; i++) {
        P.block(i*C.rows(), 0, C.rows(), _Q.cols()) = C*_Q.pow(i+1);
    }
//    OCRA_WARNING("Built P");
}

void MIQPController::buildPreviewInputMatrix(const Eigen::MatrixXd &C, Eigen::MatrixXd &R) {
    R.setZero();
    // Create first column
    Eigen::MatrixXd RColumn(C.rows()*_miqpParams.N, _T.cols());
    for (unsigned int i=0; i<_miqpParams.N; i++){
        RColumn.block(i*C.rows(), 0, C.rows(), _T.cols()) = C * _Q.pow(i) * _T;
    }
    // Shift RColumn into every column of R to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<_miqpParams.N){
        R.block(j*C.rows(), j*_T.cols(), C.rows()*(_miqpParams.N-j), _T.cols()) = RColumn.topRows((_miqpParams.N-j)*C.rows());
        j=j+1;
    }
//    OCRA_WARNING("Built R");
}

void MIQPController::buildEqualityConstraintsMatrices(const Eigen::VectorXd &x_k, Eigen::MatrixXd &Aeq, Eigen::VectorXd &Beq) {
    _Ci_eq.resize(1,STATE_VECTOR_SIZE);
    _Ci_eq << 0,0,0,0,1,-1,1,-1, Eigen::VectorXd::Zero(8);
    Aeq.setZero();

    // Create first column of Aeq
    Eigen::MatrixXd AeqColumn(_miqpParams.N, _T.cols());
    for (unsigned int i=0; i<_miqpParams.N; i++){
        AeqColumn.block(i*_Ci_eq.rows(), 0, _Ci_eq.rows(), _T.cols()) = _Ci_eq * _Q.pow(i) * _T;
    }
    // Shift AeqColumn into every column of Aeq to take the form of a lower diagonal toeplitz matrix
    unsigned int j=0;
    while (j<_miqpParams.N){
        Aeq.block(j*_Ci_eq.rows(), j*_T.cols(), _Ci_eq.rows()*(_miqpParams.N-j), _T.cols()) = AeqColumn.topRows((_miqpParams.N-j)*_Ci_eq.rows());
        j++;
    }

    // Build time-independent matrices in RHS of equality constraints.
    // First build vector of fc
    _fcbar_eq.resize(_miqpParams.N);
    _fcbar_eq.setZero();

    _rhs_2_eq.resize(_Ci_eq.rows()*_miqpParams.N, _Q.cols());
    for (unsigned int i=0; i < _miqpParams.N; i++) {
        _rhs_2_eq.block(i*_Ci_eq.rows(),0,_Ci_eq.rows(),_Q.cols()) = _Ci_eq*_Q.pow(i+1);
    }
    
    updateEqualityConstraints(x_k, Beq);
}

void MIQPController::updateEqualityConstraints(const Eigen::VectorXd &x_k, Eigen::VectorXd &Beq) {
    // Build RHS
    _Beq = _fcbar_eq - _rhs_2_eq*x_k;
}
