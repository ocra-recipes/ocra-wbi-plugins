/**
 *  \class BaseOfSupport
 *  \brief Builds base of support and corresponding constraints based on the robot's feet location.
 *  \author Jorhabib Eljaik
 *  \cite ibanezThesis2015
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *  \details This class uses the Geometry Boost libraries in order to find the
 *  vertices of the base of support (support polygon) defined by the location of
 *  the feet of the robot. Afterwards, it will build the inequality constraints
 *  that define the area of this polygon, useful to MIQPLinearConstraints to define
 *  the constraints on the Center of Pressure which should always lie within the
 *  support polygon.
 **/

#ifndef _BASE_OF_SUPPORT_H_
#define _BASE_OF_SUPPORT_H_

// Boost headers
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
// Eigen headers
#include <Eigen/Core>
#include <walking-client/MIQPState.h>
#include <walking-client/StepController.h>
#include <ocra-recipes/TaskConnection.h>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> point;
typedef boost::geometry::model::polygon<point> Polygon;

class BaseOfSupport {
private:
    /**
     * Matrix \f$\mathbf{A}_p\f$ in the inequality expressing the CoP constraints of the system at the current
     * time instant \f$k\f$ and given by \f$\mathbf{A}_p \mathbf{\xi}_k \leq \mathbf{f}_p\f$. \f$\mathbf{A}_p\f$
     * is a \f$n_p \times 16\f$ matrix and \f$\mathbf{f}_p\f$ a vector in \f$\mathbb{R}^{n_p}\f$
     */
    Eigen::MatrixXd _A;
    /**
     * Vector \f$\mathbf{f}_p\f$ in the inequality expressing the CoP constraints of the system at the current
     * time instant \f$k\f$ and given by \f$\mathbf{A}_p \mathbf{\xi}_k \leq \mathbf{f}_p\f$ 
     */    
    Eigen::VectorXd _B;
    /**
     * Polygon object holding the current points of contact.
     */
    Polygon _poly;
    /**
     * Polygon object representing the convex hull delimiting #_poly
     */
    Polygon _hull;
    /**
     * Pointer to the `stepController` object instantiated by the `walking-client`. Since it contains all the contact
     * tasks, it was easier to interface to it to requests the current contact state of the robot, i.e. contact points 
     * locations on the ground. 
     * 
     * @warning This is not thread safe yet. It might be better to do this in a different way.
     */
    std::shared_ptr<StepController> _stepController;
public:
    /**
     * Constructor. 
     * @param stepController Pointer to the stepController object instantiated by `walking-client`
     * 
     * @warning Need to make this thread-safe as this class is running in a different thread from `walking-client`'s
     */
    BaseOfSupport(std::shared_ptr<StepController> stepController);
    
    /**
     * Destructor
     */
    virtual ~BaseOfSupport ();
    
    /**
     * To be called at every update cycle of the hosting client. 
     * 
     * Calls computeConvexHull() and StepController::getContact2DCoordinates(), then 
     * uses the current system state #xi_k to finally compute #_A and #_B.
     * 
     * @see #_A, #_B
     */
    bool update(Eigen::VectorXd xi_k);
    
    /**
     * Computes the convex hull points for the current support configuration given the corners
     * of the robot's feet in #feetCorners.
     * 
     * @param feetCorners Matrix of 2D coordinates of the feet corners (or contact points).
     * 
     * @see StepController::getContact2DCoordinates()
     */
    Eigen::MatrixXd computeConvexHull(Eigen::MatrixXd &feetCorners);
    
    /**
     * Getter for the inequality matrix A
     * 
     * @see #_A
     */
    void getA(Eigen::MatrixXd &A);
    
    /**
     * Getter for the inequality vector b
     * 
     * @see #_B
     */
    void getB(Eigen::VectorXd &B);
    
protected:
    void computeAandB(Eigen::MatrixXd &listConvexHull,
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
