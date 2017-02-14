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
    /** 
     * Constructor.
     *
     * @param period        Thread period in ms
     * @param params        MIQP parameters
     * @params robotModel   Pointer to the robot model instantiated by the containing client (walking-client)
     * @param comStateRef   Reference to a matrix of CoM state references
     */
    MIQPController(int period, MIQPParameters params, ocra::Model::Ptr robotModel, const Eigen::MatrixXd &comStateRef);
    
    /**
     * Destructor
     */
    virtual ~MIQPController();
    
    /**
     * Performs all the initialization of the MIQP controller such as:
     * - Initializing the Gurobi model.
     * - Set lower and upper bounds.
     * - Create the LTI matrices of the constraints (inequality and equality constraints).
     * - Set variable types.
     * - Setup the eigen-gurobi object.
     */
    virtual bool threadInit();
    
    /**
     * Deallocates in memory. In particular, that of the Gurobi environment.
     */
    virtual void threadRelease();
    
    /**
     * Main loop of this thread. Performs the following operations:
     * - Updates the state vector.
     * - Updates the state-dependent RHS of the constraints.
     * - Sets the "moving" CoM reference in the current preview window.
     * - Solves the optimization problem.
     * - Retrieves the solution.
     */
    virtual void run();

    /**
     Takes N elements from an original trajectory of desired CoM states from state k

     @param comStateRef Sets _H_N_r for a preview window of size N from time k
     */
    void setCOMStateRefInPreviewWindow(unsigned int k, Eigen::VectorXd &comStateRef);

protected:
    // MARK: - PROTECTED METHODS
    // TODO: Document
    void setLinearPartObjectiveFunction();

    // FIXME: Deprecate
    /**
     Sets the objective function object for Gurobi.
     /warning To be deprecated if eigen-gurobi works fine
     */
    void setQuadraticPartObjectiveFunction();

    /**
     Technically should set the constraints matrix _Aineq.
     /warning To be deprecated if eigen-gurobi works fine
     */
    void setConstraintsMatrix();

    // FIXME: Deprecate
    /**
     /warning To be deprecated if eigen-gurobi works fine

     @param[out] variablesTypes Vector of variables' types
     */
    void setVariablesTypes(char * variablesTypes);

    // FIXME: Deprecate
    /**
     Sets the objective function with only the quadratic term.
     @warning To be deprecated if eigen-gurobi works fine
     */
    void setObjectiveFunction();

    // FIXME: Deprecate
    /**
     Adds variables to the model. Sets also the variable types and calls the lower and upper bounds method

     @return Pointer to object of Gurobi variables.
     @warning To be deprecated if eigen-gurobi works fine
     */
    GRBVar* addVariablesToModel();

    /**
     Sets MIQPController::_lb and MIQPController::_ub \see MIQPController::_lb MIQPController::_ub
     */
    void setLowerAndUpperBounds();

    /**
     Updates the state vector \f$xi_k\f$. \see MIQPController::xi_k
     */
    void updateStateVector();

    /**
     *  Builds \f$A_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Ah.
     *  @see ZmpPreviewController::Ah
     */
    Eigen::MatrixXd buildAh(int dt);

    /**
     *  Builds \f$B_h\f$. Called during the member list initialization of the constructor of this class.
     *
     *  @param dt Thread period in which this classed in instantiated.
     *  @return The constant matrix Bh.
     *  @see ZmpPreviewController::Bh
     */
    Eigen::MatrixXd buildBh(int dt);

    /**
     Builds the matrix \f$\mathbf{Q}\f$. \see MIQPController::Q

     @return Matrix Q
     */
    Eigen::MatrixXd buildQ();

    /**
     Builds the matrix \f$\mathbf{T}\f$. \see MIQPController::T

     @return Matrix T
     */
    Eigen::MatrixXd buildT();
    // TODO: Document
    Eigen::MatrixXd buildC_H();
    // TODO: Document
    Eigen::MatrixXd buildC_P();
    // TODO: Document
    Eigen::MatrixXd buildC_B();
    // TODO: Document
    Eigen::MatrixXd buildH_N();
    // TODO: Document
    Eigen::MatrixXd buildNb();
    // TODO: Document
    Eigen::MatrixXd buildSw();
    // TODO: Document
    Eigen::MatrixXd buildPreviewStateMatrix(Eigen::MatrixXd C);
    // TODO: Document
    Eigen::MatrixXd buildPreviewInputMatrix(Eigen::MatrixXd C);

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
     *   0  &     \mathbf{I}_2     &  \delta t     \\
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

    /* Matrix Q in \f$ \underset{x}{\text{min}}\; x^TQx^T + c^Tx \f$
     *
     * \f[
     \mathbf{Q} = \left[\begin{array}{cc}
     \mathbf{I}_{10\times10} & \mathbf{0}_{10\times6}\\
     \mathbf{0}_{6\times10} & \mathbf{A_h}_{6\times6}
     \end{array}\right]
     * \f]
     *
     * @see MIQPController::_Ah
     */
    Eigen::MatrixXd _Q;

    /* Matrix T in \f$ \underset{x}{\text{min}}\; x^TQx^T + c^Tx \f$
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

    // TODO: Document
    Eigen::MatrixXd _C_H;
    // TODO: Document
    Eigen::MatrixXd _C_P;
    // TODO: Document
    Eigen::MatrixXd _C_B;
    // TODO: Document
    Eigen::MatrixXd _P_H;
    // TODO: Document
    Eigen::MatrixXd _P_P;
    // TODO: Document
    Eigen::MatrixXd _P_B;
    // TODO: Document
    Eigen::MatrixXd _R_H;
    // TODO: Document
    // 2N * 12N
    Eigen::MatrixXd _R_P;

    // TODO: Document
    Eigen::MatrixXd _R_B;
    // TODO: Document
    Eigen::MatrixXd _H_N;
    // TODO: Document
    Eigen::MatrixXd _Nb;
    // TODO: Document
    Eigen::MatrixXd _Sw;


    /** Inequality matrix of the MIQP problem, i.e.
      * \f[
      *  A_{\text{ineq}} x \leq b_{\text{ineq}}
      * \f]
      */
    Eigen::MatrixXd _Aineq;

    /** RHS inequality matrix \f$b_{\text{ineq}}\f$ of the MIQP problem, i.e.
      * \f[
      *  A_{\text{ineq}} x \leq b_{\text{ineq}}
      * \f]
      * */
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
