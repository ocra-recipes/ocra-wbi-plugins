#ifndef OCRA_WBI_CONVERSIONS_H
#define OCRA_WBI_CONVERSIONS_H

#include <iostream>
#include <map>
#include <vector>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Log.h>
#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <Eigen/Lgsm>


namespace ocra_yarp{

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRm;

    /* Conversions between types/conventions used in Eigen and WBI */
    class OcraWbiConversions
    {
    public:
        static bool eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame);
        static bool wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp);
        static bool wbiToOcraTwistVector(Eigen::Twistd &t_wbi, Eigen::Twistd &t_ocra);
        static bool wbiToOcraSegJacobian(const Eigen::MatrixXd &jac, Eigen::MatrixXd &J);
        static bool wbiToOcraCoMJacobian(const Eigen::MatrixXd &jac, Eigen::Matrix<double,3,Eigen::Dynamic> &J);
        static bool eigenRowMajorToColMajor(const MatrixXdRm &M_rm, Eigen::MatrixXd &M);
        static bool wbiToOcraMassMatrix(int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_ocra);
        static bool wbiToOcraBodyVector(int qdof, const Eigen::VectorXd &v_wbi, Eigen::VectorXd &v_ocra);
        static bool eigenToYarpVector(const Eigen::VectorXd &eigenVector, yarp::sig::Vector &yarpVector);


    };
}
#endif
