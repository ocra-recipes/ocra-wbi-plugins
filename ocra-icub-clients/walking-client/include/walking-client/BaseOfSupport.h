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
    Eigen::MatrixXd _A;
    Eigen::VectorXd _B;
    Polygon _poly;
    Polygon _hull;
    StepController _stepController;
public:
    BaseOfSupport(StepController stepController);
    virtual ~BaseOfSupport ();

    void computeBoSCenter(Eigen::Vector2d &r);
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
    bool update(Eigen::VectorXd xi_k);
    Eigen::MatrixXd computeConvexHull(Eigen::MatrixXd &feetCorners);
    Eigen::MatrixXd getFeetCorners(int gamma);
};

#endif
