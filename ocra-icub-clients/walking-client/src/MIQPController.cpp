#include "walking-client/MIQPController.h"


MIQPController::MIQPController(int period, MIQPParameters params, ocra::Model::Ptr robotModel, const Eigen::MatrixXd &comStateRef) : RateThread(period),
_robotModel(robotModel),
_miqpParams(params),
_comStateRef(comStateRef),
_period(period),
_env(0),
_model(0),
_obj(0),
_vars(0),
_lb(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_ub(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_xi_k(Eigen::VectorXd(STATE_VECTOR_SIZE)),
_X_kn(Eigen::VectorXd(INPUT_VECTOR_SIZE*_miqpParams.N)),
_Ah(Eigen::MatrixXd(6,6)),
_Bh(Eigen::MatrixXd(6,2)),
_Q(Eigen::MatrixXd::Identity(SIZE_STATE_VECTOR, SIZE_STATE_VECTOR)),
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
_H_N_r(6*_miqpParams.N)

{
    buildAh(_period, _Ah);
    buildBh(period, _Bh);
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
    buildH_N(_H_N);
    _k = 0;
}

MIQPController::~MIQPController() {
//    OCRA_WARNING("Built an MIQP Controller object");
}

bool MIQPController::threadInit() {

    // Instantiate MIQP state object
    _state = std::make_shared<MIQPState>(_robotModel);
    
    // Sets lower and upper bounds
    setLowerAndUpperBounds();

    // Instantiate MIQPLinearConstraints object and update constraints matrix _Aineq
    _constraints = std::make_shared<MIQPLinearConstraints>(_period, _miqpParams.N);
    _Aineq.resize(_constraints->getTotalNumberOfConstraints(),  SIZE_INPUT_VECTOR * _miqpParams.N );
    _constraints->getConstraintsMatrixA(_Aineq);

    // Resize _Bineq
    _Bineq.resize(_Aineq.rows());

    // FIXME: There is at least one equality constraint for Simultaneity. Fix this!
    // Setup eigen-gurobi object with 12*N variables, 0 equality constraints and rows-of-Aineq inequality constraints.
    try {
        _eigGurobi.problem(INPUT_VECTOR_SIZE*_miqpParams.N, 0, _Aineq.rows());

    // In the previous initialization all variables are assumed continuous by default. Specify which ones are binary (4->9) i.e. alpha_x, alpha_y, beta_x, beta_y, delta, gamma
    for (int i = 4; i <=9; i++)
        _eigGurobi.setVariableType(i, GRB_BINARY);
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
    // FIXME: This method needs to be implemented.
    updateStateVector();

    // Update constraints.
    // NOTE: _Aineq is time-invariant and thus built only once, while _Bineq is state dependant (also depends on a history of states when walking constraints are included).
    // FIXME: Make sure that _xi_k was updated from the MIQPState object.
    _constraints->updateRHS(_xi_k);
    _constraints->getRHS(_Bineq);
//    OCRA_WARNING("RHS Retrieved");

    // TODO: Still gotta set the equality constraints such as Simultaneity
    Eigen::VectorXd beqNULL;beqNULL.resize(0); //beqNULL.setZero();
    Eigen::MatrixXd AeqNULL;AeqNULL.resize(0,0);// AeqNULL.setZero();

    setCOMStateRefInPreviewWindow(_k, _H_N_r);
    setLinearPartObjectiveFunction();

    try {
    // TODO: Watch out! _eigGurobi will add a 1/2. Therefore the 2. Check that this is correct.
    // FIXME: The expression for _H_N is missing regularization terms
    _eigGurobi.solve(2*_H_N, _linearTermTransObjFunc, AeqNULL, beqNULL, _Aineq, _Bineq, _lb, _ub);

    // Get the solution
    _X_kn = _eigGurobi.result();
    } catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    _k++;
}

void MIQPController::setCOMStateRefInPreviewWindow(unsigned int k, Eigen::VectorXd &H_N_r) {
    unsigned int j = 0;
    // FIXME: Pass an actual reference of CoM states
    for (unsigned int i = k + 1; i <= k + _miqpParams.N; i++) {
        H_N_r.segment(j, 6) = _comStateRef.row(i);
        j += 6;
    }
}

void MIQPController::setVariablesTypes(char * variablesTypes) {

    unsigned int k = 0;
    while (k < INPUT_VECTOR_SIZE*_miqpParams.N) {
        for (unsigned int i = 0; i < 4; i++)
            variablesTypes[k+i] = GRB_CONTINUOUS;
        for (unsigned int i = 4; i < 10; i++)
            variablesTypes[k+i] = GRB_BINARY;
        for (unsigned int i = 10; i < 12; i++)
            variablesTypes[k+i] = GRB_CONTINUOUS;
        k += INPUT_VECTOR_SIZE;
    }
}

void MIQPController::setLowerAndUpperBounds() {
    // N blocks of [a_0 a_1 b_0 b_1 alpha_0 alpha_1 beta_0 beta_1 delta gamma u_x u_y]
    _lb.setZero();
    _ub.setOnes();
    unsigned int k = 0;
    while (k < INPUT_VECTOR_SIZE*_miqpParams.N) {
        for (unsigned int j = 0; j < 4; j++)
            _lb[k+j] = GRB_INFINITY;
        for (unsigned int j = 10; j < INPUT_VECTOR_SIZE; j++)
            _lb[k+j] = GRB_INFINITY;
        k += INPUT_VECTOR_SIZE;
    }
    k = 0;
    while (k < INPUT_VECTOR_SIZE*_miqpParams.N) {
        for (unsigned int j = 0; j < 4; j++)
            _ub[k+j] = GRB_INFINITY;
        for (unsigned int j = 10; j < INPUT_VECTOR_SIZE; j++)
            _ub[k+j] = GRB_INFINITY;
        k += INPUT_VECTOR_SIZE;
    }
}

void MIQPController::updateStateVector() {
    _state->updateStateVector();
    _state->getFullState(_xi_k);
}

GRBVar* MIQPController::addVariablesToModel() {
    // TODO: Double check this
//    std::string variablesNames[INPUT_VECTOR_SIZE] = {"a_0", "a_1", "b_0", "b_1", "alpha_0", "alpha_1", "beta_0", "beta_1", "delta", "gamma", "u_x", "u_y"};
    // Set variables types
    char variablesTypes[INPUT_VECTOR_SIZE*_miqpParams.N];
    setVariablesTypes(&variablesTypes[0]);

    // Set lower and upper bounds
    setLowerAndUpperBounds();

    // Add variables
    return this->_model->addVars(_lb.data(), _ub.data(), NULL, variablesTypes, NULL, INPUT_VECTOR_SIZE);
}

void MIQPController::setObjectiveFunction() {
    // Set quadratic part
    setQuadraticPartObjectiveFunction();
    // Set linear part
//    setLinearPartObjectiveFunction();
    // Add quadratic part to objective function

}

void MIQPController::setConstraintsMatrix() {

}

void MIQPController::setQuadraticPartObjectiveFunction() {
    unsigned int cols = INPUT_VECTOR_SIZE*_miqpParams.N;
    // This part of the cost function is actually time-invariant
    // TODO: Check if the indexing of the Eigen matrix _H_N corresponds to the logic here implemented for X^T * H_N * X
    for (unsigned int i = 0; i < cols; i++)
        for (unsigned int j = 0; j < cols; j++)
            if (_H_N(i*cols+j) != 0)
                _obj += _H_N(i*cols+j)*_vars[i]*_vars[j];
}

void MIQPController::setLinearPartObjectiveFunction() {
    _linearTermTransObjFunc = -2*(_H_N_r - _P_H*_xi_k).transpose()*_Sw*_R_H + 2*((_P_P - _P_B)*_xi_k).transpose()*_Nb*(_R_P - _R_B);
//    OCRA_WARNING("Set Linear Part of the Obj Function");
}

void MIQPController::buildAh(int dt, Eigen::MatrixXd &Ah) {
    Ah.setIdentity();
    dt = dt/1000;
    Ah.block(0,2,2,2) = dt*Eigen::Matrix2d::Identity();
    Ah.block(0,4,2,2) = (pow(dt,2)/2)*Eigen::Matrix2d::Identity();
    Ah.block(2,4,2,2) = dt*Eigen::Matrix2d::Identity();
    // OCRA_WARNING("Built Ah");
}

void MIQPController::buildBh(int dt, Eigen::MatrixXd &Bh){
    dt = dt/1000;
    Bh << (pow(dt,3)/6)*Eigen::Matrix2d::Identity(), (pow(dt,2)/2)*Eigen::Matrix2d::Identity(), dt*Eigen::Matrix2d::Identity();
    // OCRA_WARNING("Built Bh");
}

void MIQPController::buildQ(Eigen::MatrixXd &Q) {
    Q.block(10,10,_Bh.rows(),_Bh.cols()) = _Bh;
    // OCRA_WARNING("Built Q");
}

void MIQPController::buildT(Eigen::MatrixXd &T) {
    T.setZero();
    T.block(0,0,10,10) = Eigen::MatrixXd::Identity(10, 10);
    T.block(10,10,6,2) = _Bh;
    // OCRA_WARNING("Built T");
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
    // OCRA_INFO("[ " << _R_H.cols() << " x " << _R_H.rows() << " x " << " ")
    H_N =   _R_H.transpose()*_Sw*_R_H + (_R_P - _R_B).transpose() * _Nb * (_R_P - _R_B);
//    OCRA_WARNING("Built H_N");
}

void MIQPController::buildNb(Eigen::MatrixXd &Nb) {
    Nb = Eigen::MatrixXd::Identity(2*_miqpParams.N, 2*_miqpParams.N);
//    OCRA_WARNING("Built Nb");
}

void MIQPController::buildSw(Eigen::MatrixXd &Sw) {
    // FIXME: For the time being this is hardcoded
    Sw = Eigen::MatrixXd::Identity(6*_miqpParams.N, 6*_miqpParams.N);
//    OCRA_WARNING("Built Sw");
}


void MIQPController::buildPreviewStateMatrix(Eigen::MatrixXd &C, Eigen::MatrixXd &P) {
    P.setZero();
    for (unsigned int i=0; i<_miqpParams.N; i++) {
        P.block(i*C.rows(), 0, C.rows(), _Q.cols()) = C*_Q.pow(i+1);
    }
//    OCRA_WARNING("Built P");
}

void MIQPController::buildPreviewInputMatrix(Eigen::MatrixXd &C, Eigen::MatrixXd &R) {
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
