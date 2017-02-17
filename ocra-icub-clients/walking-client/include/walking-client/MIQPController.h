/*! \file       MIQPController.h
 *  \brief      A thread for launching trajectory generators.
 *  \details
 *  \author     [Jorhabib Eljaik](https://github.com/jeljaik)
 *  \date       Feb 2017
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-recipes.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MIQP_CONTROLLER_H_
#define _MIQP_CONTROLLER_H_

#include "gurobi_c++.h"
#include <walking-client/utils.h>
#include <ocra-icub/OcraWbiModel.h>
#include <yarp/os/RateThread.h>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include "unsupported/Eigen/MatrixFunctions"
#include <walking-client/constraints/MIQPLinearConstraints.h>
#include "Gurobi.h" // eigen-gurobi

#define INPUT_VECTOR_SIZE 12
#define STATE_VECTOR_SIZE 16

class MIQPController : public yarp::os::RateThread {
public:
    /*
     * Constructor.
     *
     * @param period        Thread period in ms
     * @param params        MIQP parameters
     * @params robotModel   Pointer to the robot model instantiated by the containing client 
     *                      (walking-client)
     * @param comStateRef   Reference to a matrix of CoM state references. The k-th row contains the
     *                      desired CoM state at time k
     */
    MIQPController(int period, MIQPParameters params, ocra::Model::Ptr robotModel, const Eigen::MatrixXd &comStateRef);

    /*
     * Destructor
     */
    virtual ~MIQPController();

    /*
     * Performs all the initialization of the MIQP controller such as:
     * - Initializing the Gurobi model.
     * - Set lower and upper bounds of the optimization variables.
     * - Create the LTI matrices of the constraints (inequality and equality constraints).
     * - Set variable types.
     * - Setup the eigen-gurobi object.
     */
    virtual bool threadInit();

    /*
     * Deallocates in memory. In particular, that of the Gurobi environment.
     */
    virtual void threadRelease();

    /*
     * Main loop of this thread. Performs the following operations:
     * - Updates the state vector.
     * - Updates the state-dependent RHS of the constraints.
     * - Sets the "moving" CoM reference in the current preview window.
     * - Solves the optimization problem.
     * - Retrieves the solution.
     *
     * @see updateStateVector(), MIQPLinearConstraints::updateRHS(), setCOMStateRefInPreviewWindow(), setLinearPartObjectiveFunction(), Eigen::GurobiDense::solve()
     * @todo updateStateVector() needs to be implemented.
     * @todo Still gotta set the equality constraints such as Simultaneity
     * @todo Watch out! _eigGurobi will add a 1/2. Therefore the 2. Check that this is correct.
     * @todo: The expression for _H_N is missing regularization terms
     */
    virtual void run();

    /*
     Takes N elements from an original trajectory of desired CoM states from state k

     @param comStateRef Sets _H_N_r for a preview window of size N from time k
     */
    void setCOMStateRefInPreviewWindow(unsigned int k, Eigen::VectorXd &comStateRef);

protected:
    // MARK: - PROTECTED METHODS
    // TODO: Document
    void setLinearPartObjectiveFunction();

    // FIXME: Deprecate
    /*
     Sets the objective function object for Gurobi.
     /warning To be deprecated if eigen-gurobi works fine
     */
    void setQuadraticPartObjectiveFunction();

    /*
     Technically should set the constraints matrix _Aineq.
     /warning To be deprecated if eigen-gurobi works fine
     */
    void setConstraintsMatrix();

    // FIXME: Deprecate
    /*
     /warning To be deprecated if eigen-gurobi works fine

     @param[out] variablesTypes Vector of variables' types
     */
    void setVariablesTypes(char * variablesTypes);

    // FIXME: Deprecate
    /*
     Sets the objective function with only the quadratic term.
     @warning To be deprecated if eigen-gurobi works fine
     */
    void setObjectiveFunction();

    // FIXME: Deprecate
    /*
     Adds variables to the model. Sets also the variable types and calls the lower and upper bounds method

     @return Pointer to object of Gurobi variables.
     @warning To be deprecated if eigen-gurobi works fine
     */
    GRBVar* addVariablesToModel();

    /*
     * Sets MIQPController::_lb and MIQPController::_ub \see MIQPController::_lb MIQPController::_ub
     */
    void setLowerAndUpperBounds();

    /*
     * Updates the state vector \f$xi_k\f$. \see MIQPController::xi_k
     *
     * @todo Update state vector using _robotModel
     * @todo Update upper bounds
     * @todo Update rising/falling edges
     * @todo Update SS/DS
     * @todo Update potential change from DS to SS
     * @see MIQPController::_xi_k
     */
    void updateStateVector();

    /*
     *  Builds \f$A_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classe in instantiated.
     *  @return The constant matrix Ah.
     *  @see ZmpPreviewController::Ah
     */
    void buildAh(int dt, Eigen::MatrixXd &output);

    /*
     *  Builds \f$B_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Bh.
     *  @see ZmpPreviewController::Bh
     */
    void buildBh(int dt, Eigen::MatrixXd &output);

    /*
     Builds the matrix \f$\mathbf{Q}\f$. \see MIQPController::_Q

     @return Matrix Q
     */
    void buildQ(Eigen::MatrixXd &output);

    /*
     Builds the matrix \f$\mathbf{T}\f$. \see MIQPController::_T

     @return Matrix T
     */
    void buildT(Eigen::MatrixXd &output);
    
    
    /*
     * Builds output matrix of the CoM state representation.

     @param output <#output description#>
     */
    void buildC_H(Eigen::MatrixXd &output);
    // TODO: Document
    void buildC_P(Eigen::MatrixXd &output);
    // TODO: Document
    void buildC_B(Eigen::MatrixXd &output);

    /* Matrix Q in \f$ \underset{x}{\text{min}}\; x^TQx^T + c^Tx \f$
     *
     */
    void buildH_N(Eigen::MatrixXd &output);
    // TODO: Document
    void buildNb(Eigen::MatrixXd &output);
    // TODO: Document
    void buildSw(Eigen::MatrixXd &output);
    // TODO: Document
    void buildPreviewStateMatrix(Eigen::MatrixXd &C, Eigen::MatrixXd &P);
    // TODO: Document
    void buildPreviewInputMatrix(Eigen::MatrixXd &C, Eigen::MatrixXd &R);

private:
    // MARK: - PRIVATE VARIABLES
    // TODO: Document
    ocra::Model::Ptr _robotModel;
    // TODO: Document
    MIQPParameters _miqpParams;

    /** Matrix of CoM state references. Every row corresponds to the CoM state at a given time step. */
    Eigen::MatrixXd _comStateRef;

    /** Thread period in milliseconds */
    int _period;

    /** Gurobi environment. Necessary before setting up the gurobi model */

    GRBEnv * _env;

    /**
     * Gurobi model. It is built by adding variables and constraints to it and it can
     * be asked to perform the optimization or integrate model changes.
     */
    GRBModel * _model;

    // FIXME: Deprecate. As this is created by eigen-gurobi as well
    /**
     * Objective function as a GRBQadExpr object. Consists of a linear expression, plus a list of
     * coefficient-variable-variable triples that capture the quadratic terms.
     */
    GRBQuadExpr _obj;

    // FIXME: Deprecate
    GRBVar * _vars;

    /**
     *  Containts the state-dependent linear term of the objective function. This vector is to be passed to the eigen-gurobi problem.
     */
    Eigen::VectorXd _linearTermTransObjFunc;

    // FIXME: Deprecate
    Eigen::VectorXd _quadraticTermObjFunc;

    /** Vector of lower bounds of the input vector \f$\mathcal{X}\f$ */
    Eigen::VectorXd _lb;

    /** Vector of upper bounds of the input vector \f$\mathcal{X}\f$ */
    Eigen::VectorXd _ub;

    // TODO: Document
    std::string _variablesNames[INPUT_VECTOR_SIZE];

    /* State vector of the MIQP problem:
     *
     * \f[
     * \begin{array}{ccccccccc}
     *  [\mathbf{a} & \mathbf{b} & \mathbf{\alpha} & \mathbf{\beta} & \delta & \gamma & \mathbf{h} & \dot{\mathbf{h}} & \ddot{\mathbf{h}}]
     * \end{array}
     * \f]
     *
     * Where \f$\mathbf{a} \in \mathbb{R}^2\f$ are the upper bounds of the base of support (BoS), \f$b \in \mathbb{R}^2\f$ are
     * the lower bounds, \f$\mathbf{\alpha} \in \mathbb{R}^2\f$ the rising edges of \f$\mathbf{a}\f$, \f$\mathbf{\beta} \in \mathbb{R}^2\f$ are the falling edges of \f$\mathbf{b}\f$, \f$\delta\f$ indicates the potential change from double support
     * to single support, while \f$\gamma\f$ indicates whether the robot is in single support (SS) or double support (DS).
     */
    Eigen::VectorXd _xi_k;

    /* Solution \f$\mathcal{X_{k,N}}\f$ of the MIQP problem
     */
    Eigen::VectorXd _X_kn;

    /**
     *  State matrix \f$A_h\f$ from the CoM jerk integration scheme.
     *
     *  \f[
     *  \mathbf{h}_{k+1} = \mathbf{A}_h \mathbf{h}_k + \mathbf{B}_h \mathbf{u}_k
     *  \f]
     *
     *  It is a constant matrix of size \f$6\times6\f$ equal to:
     *  \f[
     *  \mathbf{A_h} = \left[ \begin{array}{ccc}
     *  \mathbf{I}_2  & \delta t \mathbf{I}_2  &  \frac{\delta t^2}{2} \mathbf{I}_2 \\
     *   0  &     \mathbf{I}_2     &  \delta t \mathbf{I}_2    \\
     *   0  &     0     &      \mathbf{I}_2
     *  \end{array} \right]
     *  \f]
     */
    Eigen::MatrixXd _Ah;

    /**
     *  Input Matrix \f$B_h\f$ from the CoM jerk integration scheme.
     *
     *  \f[
     *  \mathbf{h}_{k+1} = \mathbf{A}_h \mathbf{h}_k + \mathbf{B}_h \mathbf{u}_k
     *  \f]
     *
     *  It is constant of size \f$6\times2\f$ and equal to:
     *
     *  \f[
     *  \mathbf{B_h} = \left[ \begin{array}{c}
     *  \frac{\delta^3}{6}\mathbf{I}_2 \\
     *  \frac{\delta t^2}{2} \mathbf{I}_2 \\
     *  \delta t \mathbf{I}_2
     *  \end{array} \right]
     *  \f]
     */
    Eigen::MatrixXd _Bh;

    /*
     * Matrix \f$\mathbf{Q}\f$ in preview state model:
     * \f[
     \mathbf{\xi}_{k+1|k} = \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k}
     * \f]
     * \f[
     \mathbf{Q} = \left[\begin{array}{cc}
     \mathbf{I}_{10\times10} & \mathbf{0}_{10\times6}\\
     \mathbf{0}_{6\times10} & \mathbf{A_h}_{6\times6}
     \end{array}\right]
     * \f]
     */
    Eigen::MatrixXd _Q;

    /*
     * Matrix \f$\mathbf{T}\f$ in preview state model:
     * 
     \f[
     \mathbf{\xi}_{k+1|k} = \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k}
     \f]
     *
     \f[
     \mathbf{T} = \left[\begin{array}{cc}
     \mathbf{I}_{10\times10} & \mathbf{0}_{10\times2}\\
     \mathbf{0}_{6\times10} & \mathbf{B_h}_{6\times2}
     \end{array}\right]
     \f]
     *
     * @see MIQPController::_Bh
     */
    Eigen::MatrixXd _T;

    /**
     * Output matrix \f$\mathbf{C}_H\f$ of the CoM state space representation
     *
     * \f{align*}
     * \mathbf{\xi}_{k+1|k} &= \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k} \\
     * \hat{\mathbf{h}}_k & =\mathbf{C}_H \xi_{k|k}
     * \f}
     *
     * Where
     *
     * \f[
     *  \mathbf{C}_H = \left[
     *  \begin{array}{cc}
     *  \mathbf{0}_{6\times10} & \mathbf{I}_{6\times6}
     *  \end{array} \right]
     * \f]
     */
    Eigen::MatrixXd _C_H;

    /**
     * Output matrix \f$\mathbf{C_P}\f$ of the CoP state space representation
     *
     * \f{align*}
     * \mathbf{\xi}_{k+1|k} &= \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k} \\
     * \mathbf{p}_{k|k} & =\mathbf{C}_P \xi_{k|k}
     * \f}
     *
     * Where
     * 
     * \f[
     * \mathbf{C}_P = \left[
     \begin{array}{ccccc}
     \mathbf{0}_{2\times10} & \mathbf{I}_{2\times2} & \mathbf{0}_{2\times2} & -\frac{c_z}{g}\mathbf{I}_{2\times2}
     \end{array}\right]
     * \f]
     */
    Eigen::MatrixXd _C_P;

    /** 
     * Output matrix \f$\mathbf{C_B}\f$ of the BoS state space representation
     * 
     * \f{align*}
     * \mathbf{\xi}_{k+1|k} &= \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k} \\
     * \mathbf{r}_{k|k} & =\mathbf{C}_B \xi_{k|k}
     * \f}
     *
     * Where 
     * 
     * \f[
     \mathbf{C}_B &= \frac{1}{2} \left[
     \begin{array}{ccccc}
     \mathbf{I}_{2\times2} & \mathbf{I}_{2\times2} & \mathbf{0}_{2\times12}
     \end{array}\right]
     * \f]
     */
    Eigen::MatrixXd _C_B;

    /**
     * Matrix \f$\mathbf{P}_H\f$ in the equation for the preview CoM output matrix \f$\mathbf{H}_{k,N} = \left\{ \mathbf{h}_{k+1|k}, \mathbf{h}_{k+2|k}, \dots, \mathbf{h}_{k+N|k} \right\}\f$.
     *
     * For a preview window of size \f$N\f$:
     \f[
     \mathbf{H}_{k,N} = \mathbf{P}_H \mathbf{\xi}_k + \mathbf{R}_H \mathcal{X}_{k,N}
     \f]
     * Where:
     \f{align*}
     \mathbf{P}_H = \left[\begin{array}{c}
     \mathbf{C}_H \mathbf{Q} \\
     \vdots\\
     \mathbf{C}_H \mathbf{Q}^N
     \end{array}\right]
     \f}
     */
    Eigen::MatrixXd _P_H;
    
    /**
     * Matrix \f$\mathbf{P}_p\f$ in the equation for the preview CoP output matrix \f$\mathbf{P}_{k,N} = \left\{ \mathbf{p}_{k+1|k}, \mathbf{p}_{k+2|k}, \dots, \mathbf{p}_{k+N|k} \right\}\f$.
     *
     * For a preview window of size \f$N\f$:
     \f[
     \mathbf{P}_{k,N} = \mathbf{P}_p \mathbf{\xi}_k + \mathbf{R}_p \mathcal{X}_{k,N}
     \f]
     * Where:
     \f{align*}
     \mathbf{P}_P = \left[\begin{array}{c}
     \mathbf{C}_P \mathbf{Q} \\
     \vdots\\
     \mathbf{C}_P \mathbf{Q}^N
     \end{array}\right]
     \f}
     */
    Eigen::MatrixXd _P_P;

    /**
     * Matrix \f$\mathbf{P}_B\f$ in the equation for the preview center of BoS output matrix \f$\mathbf{R}_{k,N} = \left\{ \mathbf{r}_{k+1|k}, \mathbf{r}_{k+2|k}, \dots, \mathbf{r}_{k+N|k} \right\}\f$
     * 
     * For a preview window of size \f$N\f$:
     * 
     \f[
         \mathbf{R}_{k,N} = \mathbf{R}_B \mathbf{\xi}_k + \mathbf{R}_B \mathcal{X}_{k,N}
     \f]
     * Where:
     \f[
     \mathbf{P}_B = \left[\begin{array}{c}
     \mathbf{C}_B \mathbf{Q} \\
     \vdots\\
     \mathbf{C}_B \mathbf{Q}^N
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _P_B;
    
    /**
     * Input matrix \f$\mathbf{R}_H\f$ in the equation for the preview CoM output matrix \f$\mathbf{H}_{k,N} = \left\{ \mathbf{h}_{k+1|k}, \mathbf{h}_{k+2|k}, \dots, \mathbf{h}_{k+N|k} \right\}\f$.
     *
     * For a preview window of size \f$N\f$:
     \f[
         \mathbf{H}_{k,N} = \mathbf{P}_H \mathbf{\xi}_k + \mathbf{R}_H \mathcal{X}_{k,N}
     \f]
     * Where:
     \f[
     \mathbf{R}_h = \left[\begin{array}{cccc}
     \mathbf{C}_H\mathbf{T}               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_H\mathbf{Q}\mathbf{T}   &   \mathbf{C}_H\mathbf{T}   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_H\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_H\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_H\mathbf{T}
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _R_H;

    /**
     * Matrix \f$\mathbf{R}_p\f$ in the equation for the preview CoP output matrix \f$\mathbf{P}_{k,N} = \left\{ \mathbf{p}_{k+1|k}, \mathbf{p}_{k+2|k}, \dots, \mathbf{p}_{k+N|k} \right\}\f$.
     *
     * For a preview window of size \f$N\f$:
     \f[
     \mathbf{P}_{k,N} = \mathbf{P}_p \mathbf{\xi}_k + \mathbf{R}_p \mathcal{X}_{k,N}
     \f]
     *
     * Size: [2N * 12N]
     * Where:
     \f[
     \f]
     */
    Eigen::MatrixXd _R_P;

    /**
     * Matrix \f$\mathbf{R}_B\f$ in the equation for the preview center of BoS output matrix \f$\mathbf{R}_{k,N} = \left\{ \mathbf{r}_{k+1|k}, \mathbf{r}_{k+2|k}, \dots, \mathbf{r}_{k+N|k} \right\}\f$
     *
     * For a preview window of size \f$N\f$:
     *
     \f[
     \mathbf{R}_{k,N} = \mathbf{R}_B \mathbf{\xi}_k + \mathbf{R}_B \mathcal{X}_{k,N}
     * Where:
     *
     \f[
     \mathbf{R}_B = \left[\begin{array}{cccc}
     \mathbf{C}_B\mathbf{T}               &   0                          &  \cdots   &   0 \\
     \mathbf{C}_B\mathbf{Q}\mathbf{T}   &   \mathbf{C}_B\mathbf{T}   &  \cdots   &   0 \\
     \vdots                                 & \vdots                       & \ddots    &  \vdots \\
     \mathbf{C}_B\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_B\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_B\mathbf{T}
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _R_B;
    
    /**
     * \f$S_w\f$ is a \f$6\times6\f$ diagonal weighting selection matrix, defining whether position, 
     * velocity and/or acceleration of the CoM are tracked in each of the two horizontal directions.
     *
     * \todo For the moment this has been hardcoded! Put it in configuration file.
     */
    Eigen::MatrixXd _Sw;

    /**
     * \f$N_b\f$ is a diagonal weighting matrix whose scalar weights help define the compromise between balance robustness and tracking performance.
     */
    Eigen::MatrixXd _Nb;
    
    /**
     * Positive definite matrix \f$H_N\f$ with the coefficients of the quadratic objective of the MIQP for the walking MPC problem.
     *
     * For the time being this is:
     *
     * \f[
     *   \mathbf{R}_H^T \mathbf{S}_w \mathbf{R}_H + (\mathbf{R}_P - \mathbf{R}_B)^T \mathbf{N}_b (\mathbf{R}_P - \mathbf{R}_B)
     * \f]
     * But this expression is missing the regularizing terms of the cost function. Current size is \f$12N\times12N\f$
     */
    Eigen::MatrixXd _H_N;


    /** Inequality matrix of the MIQP problem, i.e.
      * \f[
      *  A_{\text{ineq}} x \leq b_{\text{ineq}}
      * \f]
      *
      * Set by MIQPLinearConstraints::getConstraintsMatrixA().
      */
    Eigen::MatrixXd _Aineq;

    /** RHS inequality vector \f$b_{\text{ineq}}\f$ of the MIQP problem, i.e.
      * \f[
      *  A_{\text{ineq}} x \leq b_{\text{ineq}}
      * \f]
      *
      * Set by MIQPLinearConstraints::getRHS()
      */
    Eigen::VectorXd _Bineq;

    /* eigen-gurobi object */
    Eigen::GurobiDense _eigGurobi;

    /* Linear constraints object */
    std::shared_ptr<MIQPLinearConstraints> _constraints;

    /* Reference CoM state in preview window. Size: [6N]*/
    Eigen::VectorXd _H_N_r;

    /* Current iteration */
    unsigned int _k;
};

#endif
