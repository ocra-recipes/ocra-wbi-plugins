/**
 *  \class BaseOfSupport
 *  \brief Builds base of support and corresponding constraints based on the robot's feet location.
 *  \author Jorhabib Eljaik
 *  \cite ibanezThesis2015
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *  \details This class uses the Geometry Boost libraries in order to find the
 *  vertices of the bounding box defined by the location of the feet of the robot. 
 *  Afterwards, it will build the inequality constraints in a preview window of size \f$N\f$ 
 *  delimiting the area of this polygoon where the Center of Pressure is constrained to lie.
 *  
 *  The bounding box of the Base of Support writes as a set of intequality constraints:
     \f[
     \left[\begin{array}{cc}
     -1  &  0\\
     1  &  0\\
     0  & -1\\
     0  &  1
     \end{array}\right] \mathbf{p}
     \leq
     \left[\begin{array}{c}
     -x_{\text{min}}\\
     x_{\text{max}}\\
     -y_{\text{min}}\\
     y_{\text{max}}
     \end{array}\right]
     \f]
 * or,
     \f[
     \mathbf{A}_b\mathbf{p} \leq \mathbf{b}
     \f]
 * Which in terms of the system state \f$\xi\f$ writes:
     \f[
     \left[\begin{array}{cc}
     \mathbf{0}_{10\times10} & \\
     & \mathbf{A}_b \mathbf{C}_p
     \end{array}\right] \mathbf{\xi} \leq
     \left[\begin{array}{c}
     \mathbf{0} \\
     \mathbf{b}
     \end{array}\right]
     \f]
 * or:
     \f[
     \mathbf{C}_i \mathbf{\xi} \leq \mathbf{f}
     \f]
 * Where \f$\mathbf{C}_p\f$ expresses the relationship between CoM and CoP, see buildCp() for more details.
 *
 * For this type of constraint, in a preview window of size \f$N\f$ we have:
     \f[
     \left[\begin{array}{cccc}
     \mathbf{C}_i\mathbf{T}                 & 0                        & \cdots   &   0 \\
     \mathbf{C}_i\mathbf{Q}\mathbf{T}       & \mathbf{C}_i\mathbf{T}   & \cdots   &   0 \\
     \vdots                                 & \vdots                   & \ddots    &  \vdots \\
     \mathbf{C}_i\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_i\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_i\mathbf{T}
     \end{array}\right] \mathbf{\mathcal{X}}_{N,k} =
     \left[\begin{array}{c}
     \mathbf{f}_c\\
     \mathbf{f}_c\\
     \vdots\\
     \mathbf{f}_c
     \end{array}\right] -
     \left[\begin{array}{c}
     \mathbf{C}_i\mathbf{Q}\\
     \mathbf{C}_i\mathbf{Q}^2\\
     \vdots\\
     \mathbf{C}_i\mathbf{Q}^N
     \end{array}\right] \mathbf{\xi}_k
     \f]
 * or:
     \f[
        \mathbf{A} \mathbf{\mathcal{X}}_{N,k} \leq \bar{\mathbf{f}} - \mathbf{B}\xi_k
    \f]
 *
 *
 **/

#ifndef _BASE_OF_SUPPORT_H_
#define _BASE_OF_SUPPORT_H_

// Boost headers
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include "unsupported/Eigen/MatrixFunctions"
// Eigen headers
#include <Eigen/Core>
#include <walking-client/MIQPState.h>
#include <walking-client/StepController.h>
#include <walking-client/utils.h>
#include <ocra-recipes/TaskConnection.h>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> point;
typedef boost::geometry::model::polygon<point> Polygon;
typedef boost::geometry::model::box<point> Box;

class BaseOfSupport {
private:
    /**
     * Inequality matrix \f$\mathbf{A}_b\f$ expressing the bounding box for the current contact configuration of the robot's feet
     \f[
     \mathbf{A}_b =
     \left[\begin{array}{cc}
     -1  &  0\\
     1  &  0\\
     0  & -1\\
     0  &  1
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _Ab;
    
    /**
     * Right-hand-side vector of the inequality expression the interior of the bounding box for the current
     * contact configuration of the robot's feet.
     * For the current bounding box defined by minimum and maximum points \f$\mathbf{p}_{\text{min}}\f$
     * and \f$\mathbf{p}_{\text{max}}\f$ we get:
     \f[
     \mathbf{b} =
     \left[
     \begin{array}{c}
     -x_{\text{min}}\\
     x_{\text{max}}\\
     -y_{\text{min}}\\
     y_{\text{max}}
     \end{array}
     \right]
     \f]
     */
    Eigen::VectorXd _b;

    /**
     * Output matrix \f$\mathbf{C_P}\f$ of the CoP state space representation
     *
     \f{align*}
     \mathbf{\xi}_{k+1|k} &= \mathbf{Q} \xi_{k|k} + \mathbf{T}\mathcal{X}_{k+1|k} \\
     \mathbf{p}_{k|k} & =\mathbf{C}_P \xi_{k|k}
     \f}
     *
     * Where
     *
     \f[
     \mathbf{C}_P = \left[
     \begin{array}{ccccc}
     \mathbf{0}_{2\times10} & \mathbf{I}_{2\times2} & \mathbf{0}_{2\times2} & -\frac{c_z}{g}\mathbf{I}_{2\times2}
     \end{array}\right]
     \f]
     *
     * Size: \f$[2\times16]\f$
     */
    Eigen::MatrixXd _Cp;

    /**
     * Matrix \f$\mathbf{C}_i\f$ in the inequality expressing the CoP constraints of the system at the current
     * time instant \f$k\f$ and given by \f$\mathbf{C}_i \xi_k \leq \mathbf{f}\f$.
     * \f$\mathbf{C}_i\f$ is a \f$14 \times 16\f$ matrix, and \f$\mathbf{f}_p\f$ a
     * vector in \f$\mathbb{R}^{n_p}\f$ or:
     \f[
     \left[\begin{array}{cc}
     \mathbf{0}_{10\times10} & \\
     & \mathbf{A}_b \mathbf{C}_p
     \end{array}\right] \mathbf{\xi} \leq
     \left[\begin{array}{c}
     \mathbf{0} \\
     \mathbf{b}
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _Ci;
    
    /**
     * Right-hand side vector of the inequalities expressing the bounding box, when the
     * latter is expressed in terms of the sytem state \f$\xi_k\f$ and equal to:
     *
     \f[
     \begin{array}{c}
     \left[
     \mathbf{0}\\
     \mathbf{b}
     \end{array}\right]
     \f]
     */
    Eigen::VectorXd _f;

    /**
     * Matrix \f$\mathbf{A}\f$ in the bounding box constraints for a preview window of size \f$N\f$
     \f[
     \mathbf{A} =
     \left[\begin{array}{cccc}
     \mathbf{C}_i\mathbf{T}                 & 0                        & \cdots   &   0 \\
     \mathbf{C}_i\mathbf{Q}\mathbf{T}       & \mathbf{C}_i\mathbf{T}   & \cdots   &   0 \\
     \vdots                                 & \vdots                   & \ddots    &  \vdots \\
     \mathbf{C}_i\mathbf{Q}^{N-1}\mathbf{T} & \mathbf{C}_i\mathbf{Q}^{N-2}\mathbf{T} & \cdots & \mathbf{C}_i\mathbf{T}
     \end{array}\right]
     \f]
     */
    Eigen::MatrixXd _A;
    
    /**
     * Vector \f$\bar{\mathbf{f}}\f$ in the bounding box constraints for a preview window of size \f$N\f$
     \f[
     \mathbf{f} =
     \left[\begin{array}{c}
     \mathbf{f}_c\\
     \mathbf{f}_c\\
     \vdots\\
     \mathbf{f}_c
     \end{array}\right]
     \f]
     */
    Eigen::VectorXd _fbar;
    
    /**
     * Matrix \f$\mathbf{B}\f$ appearing in the bounding box constraints for a preview window of size \f$N\f$:
     *
     \f[
         \left[\begin{array}{c}
         \mathbf{C}_i\mathbf{Q}\\
         \mathbf{C}_i\mathbf{Q}^2\\
         \vdots\\
         \mathbf{C}_i\mathbf{Q}^N
         \end{array}\right]
     \f]
     */
    Eigen::VectorXd _B;
    
    /**
     * Right-hand side of the constraints of the bounding box in a preview window of size \f$N\f$ and
     * equal to:
     \f[
         \bar{\mathbf{f}} - \mathbf{B}\xi_k
     \f]
     */
    Eigen::VectorXd _rhs;
    
    /**
     * Polygon object holding the current points of contact.
     */
    Polygon _poly;
    /**
     * Polygon object representing the bounding box (envelope) delimiting #_poly.
     */
    Box _bbox;
    /**
     * Pointer to the `stepController` object instantiated by the `walking-client`. Since it contains all the contact
     * tasks, it was easier to interface to it to requests the current contact state of the robot, i.e. contact points 
     * locations on the ground.
     * 
     * @warning This is not thread safe yet. It might be better to do this in a different way.
     */
    std::shared_ptr<StepController> _stepController;
    
    Eigen::MatrixXd _Q;
    
    Eigen::MatrixXd _T;
    
    MIQPParameters _miqpParams;
    
public:
    /**
     * Constructor. 
     * @param stepController Pointer to the stepController object instantiated by `walking-client`
     * 
     * @warning Need to make this thread-safe as this class is running in a different thread from `walking-client`'s
     */
    BaseOfSupport(std::shared_ptr<StepController> stepController, Eigen::MatrixXd Q, Eigen::MatrixXd T, MIQPParameters miqpParams);
    
    /**
     * Destructor
     */
    virtual ~BaseOfSupport ();
    
    /**
     * To be called at every update cycle of the hosting client. 
     *
     * - Retrieves the feet corners.
     * - Computes the bounding box.
     * - Updates the state-dependent right hand side of the constraints.
     * Calls computeBoundingBox() and StepController::getContact2DCoordinates(), then
     * updates #_B
     *
     * @param xi_k Current system state.
     * 
     * @see #_B
     */
    bool update(Eigen::VectorXd &xi_k);
    
    /**
     * Computes the bounding box points (minimum and maximum points of the box of the current support 
     * configuration) given the corners of the robot's feet in #feetCorners.
     * 
     * @param feetCorners Matrix of 2D coordinates of the feet corners (or contact points).
     * @param[out] minMaxBoundingBox Two row-wise points corresponding to the minimum and maximum points and in the resulting bounding box.
     * 
     * @see StepController::getContact2DCoordinates()
     */
    void computeBoundingBox(Eigen::MatrixXd &feetCorners, Eigen::Matrix2d &minMaxBoundingBox);
    
    /**
     * Getter for the inequality matrix A
     * 
     * @see #_A
     */
    void getA(Eigen::MatrixXd &A);
    
    /**
     * Getter for the inequality vector b
     * 
     * @see #_rhs
     */
    void getrhs(Eigen::VectorXd &B);
    
protected:
    void buildAb();
    void buildb(Eigen::Matrix2d minMaxBoundingBox);
    void buildCp(double cz, double g);
    void buildCi(Eigen::MatrixXd &Ab, Eigen::MatrixXd &Cp);
    void buildf();
    void buildfbar(Eigen::VectorXd &f);
    void buildB(Eigen::MatrixXd Ci, Eigen::MatrixXd Q);
    void buildA(const Eigen::MatrixXd &Ci,const Eigen::MatrixXd &Q,const Eigen::MatrixXd &T);
    void computeAandB(Eigen::Matrix2d &boundingBox,
                      Eigen::Vector2d a,
                      Eigen::Vector2d b);
    void computeAiBi(Eigen::Vector2d r,
                     Eigen::Vector2d p1,
                     Eigen::Vector2d p2,
                     Eigen::RowVector2d &Ai,
                     double &bi,
                     double tolDiff=1e-2);
    void computeMidpoint(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d &r);
};

#endif
