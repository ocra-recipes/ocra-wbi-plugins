#include "walking-client/BaseOfSupport.h"

BaseOfSupport::BaseOfSupport(StepController stepController):_stepController(stepController){
}

BaseOfSupport::~BaseOfSupport(){}

bool BaseOfSupport::update(Eigen::VectorXd xi_k) {
    Eigen::Vector2d a = xi_k.segment(MIQP::A_X,2);
    Eigen::Vector2d b = xi_k.segment(MIQP::B_X,2);
    int gamma = xi_k(MIQP::DELTA);
    // Get feet corners
    Eigen::MatrixXd feetCorners;
    feetCorners = _stepController.getContact2DCoordinates();
//    feetCorners = getFeetCorners(gamma);
    // Compute the convex hull of the current support configuration
    // Every row of listConvexHull is a 2D point of the convex hull
    Eigen::MatrixXd listConvexHull = computeConvexHull(feetCorners);
    // Resize inequality matrix _A based on the size of the convex hull
    _A.resize(listConvexHull.rows(), 2);
    _B.resize(listConvexHull.rows());
    // Compute the inequality matrices Ax <= b that represent the interior
    // of the support polygon (or BoS)
    computeAandB(listConvexHull, a, b);
    return true;
}

//Eigen::MatrixXd BaseOfSupport::getFeetCorners(int gamma) {
//    Eigen::MatrixXd feetCorners;
//    // The output could be a 4x2 matrix or an nx2 depending on whether the two feet are on the ground
//    // or just one (DS=1 or SS=0 given by gamma)
//    if (gamma) {
//        feetCorners.resize(8,2);
//        // Retrieve feet centers' location
//        // Build corners coordinates using feet dimensions and feet centers.
//        /* TODO: This can also be done by the server*/
//    } else {
//        feetCorners.resize(4,2);
//    }
//}

Eigen::MatrixXd BaseOfSupport::computeConvexHull(Eigen::MatrixXd &feetCorners){

    for (unsigned int i=0 ; i<feetCorners.rows(); i++) {
        _poly.outer().push_back(point(feetCorners(i,0), feetCorners(i,1)));
    }

    boost::geometry::convex_hull(_poly, _hull);

    using boost::geometry::dsv;
    std::cout
    << "polygon: " << dsv(_poly) << std::endl
    << "hull: " << dsv(_hull) << std::endl;
    
    // TODO: Get the points from _hull in the form of an Eigen::Matrix
//    point tmp;
//    _hull.outer().pop_back();
}

void BaseOfSupport::computeBoSCenter(Eigen::Vector2d &r)
{

}

void BaseOfSupport::computeAandB(Eigen::MatrixXd &listConvexHull,
                                 Eigen::Vector2d a,
                                 Eigen::Vector2d b)
{
    Eigen::Vector2d r;
    computeMidpoint(a,b,r);
    Eigen::RowVector2d Ai;
    double bi;
    // TODO: Make sure that listConvexHull does not include the first point twice
    // Otherwise this for-loop would go until listConvexHull.rows() - 1
    for (unsigned int i = 0; i < listConvexHull.rows(); i++) {
        Eigen::Vector2d p1 = listConvexHull.row(i);
        Eigen::Vector2d p2 = listConvexHull.row(i+1);
        computeAiBi(r, p1, p2, Ai, bi);
        _A.row(i) = Ai;
        _B(i) = bi;
    }
}

void BaseOfSupport::computeAiBi(Eigen::Vector2d r,
                                Eigen::Vector2d p1,
                                Eigen::Vector2d p2,
                                Eigen::RowVector2d &Ai,
                                double &bi,
                                double tolDiff)
{
    double p1x = p1(0);
    double p1y = p1(1);
    double p2x = p2(0);
    double p2y = p2(1);
    double m;

    if (std::abs(p2x - p1x) > tolDiff) {
        m = (p2y - p1y) /  (p2x - p1x);
        Ai << -m, 1;
        bi = p1y - m*p1x;
    } else {
        m = 1;
        Ai << -m, 0;
        bi = -m*p1x;
    }
    if (Ai*r > bi) {
        Ai = -Ai;
        bi = -bi;
    }
}

void BaseOfSupport::computeMidpoint(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d &r) {
    r = 0.5*(a+b);
}
