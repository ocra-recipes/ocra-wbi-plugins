#include "walking-client/BaseOfSupport.h"

BaseOfSupport::BaseOfSupport(std::shared_ptr<StepController> stepController):_stepController(stepController){
}

BaseOfSupport::~BaseOfSupport(){}

bool BaseOfSupport::update(Eigen::VectorXd xi_k) {
    Eigen::Vector2d a = xi_k.segment(MIQP::A_X,2);
    Eigen::Vector2d b = xi_k.segment(MIQP::B_X,2);
    // Get feet corners
    Eigen::MatrixXd feetCorners;
    feetCorners = _stepController->getContact2DCoordinates();
    // Compute the convex hull of the current support configuration
    // Every row of listConvexHull is a 2D point of the convex hull
    Eigen::MatrixXd listConvexHull = computeConvexHull(feetCorners);
    // Resize inequality matrix _A based on the size of the convex hull
    // (listConvexHull.rows() - 1) rows because the list contains the closing point 
    // last_point == first_point
    _A.resize(listConvexHull.rows()-1, 2);
    _B.resize(listConvexHull.rows()-1);
    // Compute the inequality matrices Ax <= b that represent the interior
    // of the support polygon (or BoS)
    computeAandB(listConvexHull, a, b);
    return true;
}

Eigen::MatrixXd BaseOfSupport::computeConvexHull(Eigen::MatrixXd &feetCorners){
    _poly.clear();
    _hull.clear();
    OCRA_INFO("Feet corners given by StepController are: ");
    std::cout << feetCorners << std::endl;
    for (unsigned int i=0 ; i<feetCorners.rows(); i++) {
        _poly.outer().push_back(point(feetCorners(i,0), feetCorners(i,1)));
    }
    boost::geometry::correct(_poly);
    
    boost::geometry::convex_hull(_poly, _hull);

    using boost::geometry::dsv;
//     std::cout
//     << "polygon: " << dsv(_poly) << std::endl
//     << "hull: " << dsv(_hull) << std::endl;
    
    Eigen::MatrixXd  convexHullEigen;
    convexHullEigen.resize(_hull.outer().size(),2);
    int k = 0;
//     OCRA_INFO("Points in convex hull: ");
    for (auto const &value : _hull.outer()) {
//         std::cout << "[ " << boost::get<0>(value) << ", " <<boost::get<1>(value) << " ]  "<< std::endl;
        convexHullEigen(k,0) = boost::get<0>(value);
        convexHullEigen(k,1) = boost::get<1>(value);
        k++;
    }
    return convexHullEigen;
}

void BaseOfSupport::computeAandB(Eigen::MatrixXd &listConvexHull,
                                 Eigen::Vector2d a,
                                 Eigen::Vector2d b)
{
    Eigen::Vector2d r;
    computeMidpoint(a,b,r);
    OCRA_INFO("Midpoint is: " << r);
    Eigen::RowVector2d Ai;
    double bi;
    // NOTE: Make sure that listConvexHull does not include the first point twice
    // Otherwise this for-loop would go until listConvexHull.rows() - 1
    OCRA_INFO("listConvexHull is: \n" << listConvexHull);
    for (unsigned int i = 0; i < listConvexHull.rows()-1; i++) {
        Eigen::Vector2d p1 = listConvexHull.row(i);
        Eigen::Vector2d p2 = listConvexHull.row(i+1);
        computeAiBi(r, p1, p2, Ai, bi);
        _A.row(i) = Ai;
        _B(i) = bi;
    }
//     OCRA_INFO("A: \n " << _A);
//     OCRA_INFO("B: \n " << _B);
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
