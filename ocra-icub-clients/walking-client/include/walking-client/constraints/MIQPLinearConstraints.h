/*! \file       Constraint.h
 *  \brief      A constraint base class.
 *  \details    In order to use this class instantiate an object of type
 *              MIQPLinearConstraints passing to it the period of execution of
 *              the containing thread (in ms) and the length of the preview
 *              window. At 'construction' time, shape and admissibility matrices
 *              will be built while the RHS of the total inequality constraints
 *              will be only allocated, waiting for an update by the user with
 *              the current and a history of states which can be done through
 *              the updateRHS() method. If the user requires the total constraints
 *              matrix, the latter can be queried through the method
 *              getConstraintsMatrixA(), for speed concerns, the user needs to
 *              pass a matrix to this method already allocated. This can be done
 *              by querying the resulting total number of constraints through
 *              getTotalNumberOfConstraints() to resize the rows of your output
 *              matrix, while the columns will correspond to (size of input vector
 *              times the size of the preview window). After this, the user is
 *              left with updating the RHS of the inequalities to be queried also
 *              through
 *  \warning    Currently this class is only taking into account all the shape and admissibility constraints used in Ibanes 2016 thesis, thus missing any walking constraints.
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

#ifndef _MIQP_LINEAR_CONSTRAINTS_H_
#define _MIQP_LINEAR_CONSTRAINTS_H_

#include "walking-client/constraints/ShapeConstraints.h"
#include "walking-client/constraints/AdmissibilityConstraints.h"
#include "walking-client/StepController.h"
#include "walking-client/BaseOfSupport.h"
#include "walking-client/utils.h"

class MIQPLinearConstraints {
private:
    std::shared_ptr<ShapeConstraints> _shapeCnstr;
    std::shared_ptr<AdmissibilityConstraints> _admissibilityCnstr;
    std::shared_ptr<StepController> _stepController;
    Eigen::MatrixXd _AShapeAdmiss;
    Eigen::MatrixXd _BShapeAdmiss;
    Eigen::VectorXd _fcbarShapeAdmiss;
    Eigen::MatrixXd _A;
    // f_c - B*Xi_k
    Eigen::VectorXd _rhs;
    Eigen::MatrixXd _Acr;
    Eigen::MatrixXd _Acl;
    
    /**
     * Matrix \f$\mathbf{Q}\f$ from the preview model:
     \f[
     \forall j \in \mathbb{N}^*, \xi_{k+j+1|k} = \mathbf{Q} \xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}
     \f]
     *
     \f[
     \mathbf{Q} = \left[\begin{array}{cc}
     \mathbf{0}_{10\times10} & \mathbf{0}_{10\times6}\\
     \mathbf{0}_{6\times10} & \mathbf{A_h}_{6\times6}
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _Q;
    
    /** 
     * Matrix \f$\mathbf{T}\f$ from the preview model:
     \f[
     \forall j \in \mathbb{N}^*, \xi_{k+j+1|k} = \mathbf{Q} \xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}
     \f]
     *
     \f[
     \mathbf{T} = \left[\begin{array}{cc}
     \mathbf{I}_{10\times10} & \mathbf{0}_{10\times2}\\
     \mathbf{0}_{6\times10} & \mathbf{B_h}_{6\times2}
     \end{array}\right]
     \f]
     *
     * @see #_Bh
     */

    Eigen::MatrixXd _T;

    /**
     *  Input matrix \f$\mathbf{B}_h\f$ from the linear state process of the CoMstate \f$\hat{\mathbf{h}}\f$. It is constant of size \f$6\times2\f$ and equal to:
     \f[
     \mathbf{B_h} = \left[ \begin{array}{c}
     \frac{\delta^3}{6}\mathbf{I}_2 \\
     \frac{\delta t^2}{2} \mathbf{I}_2     \\
     \delta t \mathbf{I}_2
     \end{array} \right]
     \f]
     */
    Eigen::MatrixXd _Bh;
    
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
     * Boolean too add shape constraints.
     */
    bool _addShapeCtrs;
    
    /**
     * Boolean to add admissibility constraints.
     */
    bool _addAdmissibilityCtrs;
    
    /**
     * Boolean to add CoP constraints within the convex hull defined by the feet location
     */
    bool _addCoPConstraints;
    
    /**
     * Boolean to add walking constraints.
     */
    bool _addWalkingCtrs;

    /** Period in milliseconds */
    unsigned int _dt;
    
    /** Length of preview window */
    unsigned int _N;
    
    /** Total number of constraints*/
    unsigned int _nConstraints;
    
    /** Base of support object to retrieve the corresponding inequality constraints*/
    std::shared_ptr<BaseOfSupport> _baseOfSupport;
    
    MIQPParameters _miqpParams;
public:

    /**
     * @todo Once I add walking constraints this will change to include the rows added by walking constraints
     * @param[in] stepController Pointer to StepController object which is instantiated by the hosting client.
     * @param[in] miqpParams Container of the MIQP parameters. 
     * 
     */
    MIQPLinearConstraints(std::shared_ptr<StepController> stepController, MIQPParameters &miqpParams);
    
    /**
     * Default destructor
     */
    virtual ~MIQPLinearConstraints ();
    
    /**
     * Updates the state-dependent RHS of the inequality constraints 
     * 
     * @param[in] xi_k Current state.
     */
    void updateRHS(const Eigen::VectorXd& xi_k);
    
    /**
     * Retrieves the constraints matrix \f$\mathbf{A}\f$. Before passing a matrix to copy #_A allocate the space 
     * for A by calling getTotalNumberOfConstraints() to know the number of rows, while 
     * INPUT_VECTOR_SIZE * SIZE_PREVIEW_WINDOW will be the number of columns.
     *
     * @param A Reference to matrix where the global constraints matrix _A will be copied.
     */
    void getConstraintsMatrixA(Eigen::MatrixXd &A);
    
    /**
     * Returns the total number of constraints.
     *
     * @return _nConstraints
     */
    unsigned int getTotalNumberOfConstraints();

    /**
     * Returns in rhs the right-hand size of the inequality constraints of the problem.
     * 
     * @param[out] rhs State-dependent right hand side.
     * @see updateRHS()
     */
    void getRHS(Eigen::VectorXd &rhs);

protected:
    /**
     Basically builds matrix \f$\mathbf{A}\f$ (#_AShapeAdmiss) in the partial expression for the MIQP linear constraints, containing only shape and admissibility constraints, such that:
     \f[
     \mathbf{A} \mathcal{X}_{k,N} \leq \bar{\mathbf{f}}_c - \mathbf{B} \xi_k
     \f]


     \f[
     \begin{array}{cccccc}
     A_{\text{cr}} Q^0 T  &  0  &  0  &  0  &  \dots  & 0\\
     (A_{\text{cl}}Q^0 + A_{\text{cr}}Q^1)T  &  A_{\text{cr}}Q^0 T  &  0  &  0  &  \dots & 0  \\
     (A_{\text{cl}}Q^1 + A_{\text{cr}}Q^2)T &  (A_{\text{cl}}Q^0 + A_{\text{cr}}Q^1)T  &  A_{\text{cr}} Q^0 T  &  0  &  \dots & 0  \\
     \vdots & \vdots  &    &    &   & \vdots \\
     (A_{\text{cl}}Q^{N-2} + A_{\text{cr}}Q^{N-1})T & (A_{\text{cl}}Q^{N-3} + A_{\text{cr}}Q^{N-2})T & \dots & \dots & \dots & A_{\text{cr}}Q^0T \\
     \end{array}
     \f]

     \see buildAShapeAdmiss()
     \todo When walking constraints are added, matrix _A will be a stack of the two
     */
    void buildShapeAndAdmissibilityInPreviewWindow();
    /**
     Actually builds the matrix A referred to in buildShapeAndAdmissibilityInPreviewWindow()

     \see buildShapeAndAdmissibilityInPreviewWindow
     */
    void buildAShapeAdmiss();
    /**
     Actually builds the matrix B referred to in buildShapeAndAdmissibilityInPreviewWindow()

     \see buildShapeAndAdmissibilityInPreviewWindow
     */
    void buildBShapeAdmiss();
    /**
     Actually builds the vector \f$\bar{\mathbf{f}}_c\f$ referred to in buildShapeAndAdmissibilityInPreviewWindow()
     */
    void buildFcBarShapeAdmiss();
    /**
     Stacks matrices \f$C_{i+1}\f$ (Cii) from the shape and admissibility constraints to set variable #_Acr
     */
    void setMatrixAcr();
    /**
     Stacks matrices \f$C_{i}\f$ (Ci) from the shape and admissiblity constraints to set variable #_Acl
     */
    void setMatrixAcl();
    /**
     Builds matrix Q (#_Q) from the preview model:

     \f[
     \forall j \in \mathbb{N}^*, \xi_{k+j+1|k} = \mathbf{Q} \xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}
     \f]
     *
     * @see #_Q
     */
    void buildMatrixQ();
    /**
     Builds matrix T (#_T) from the preview model:

     \f[
     \forall j \in \mathbb{N}^*, \xi_{k+j+1|k} = \mathbf{Q} \xi_{k+j|k} + \mathbf{T}\mathcal{X}_{k+j+1|k}
     \f]
     *
     * @see #_T
     */
    void buildMatrixT();
    /**
     *  Builds \f$B_h\f$ (#_Bh) called by the constructor of this class.
     *
     *  @see #_Bh
     */
    void buildBh();
    /**
     *  Builds \f$A_h\f$.
     *
     *  @see #_Ah
     */
    void buildAh();
};
#endif
