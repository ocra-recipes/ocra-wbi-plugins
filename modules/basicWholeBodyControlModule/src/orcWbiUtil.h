/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#ifndef ORCWBIUTIL_H
#define ORCWBIUTIL_H

#include <Eigen/Lgsm>
#include <wbi/wbi.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRm;

    /* Conversions between types/conventions used in Eigen and WBI */
    class orcWbiConversions
    {
    public:
        static bool eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame);
        static bool wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp);
        static bool wbiToOrcTwistVector(Eigen::Twistd &t_wbi, Eigen::Twistd &t_orc);
        static bool eigenRowMajorToColMajor(const MatrixXdRm &M_rm, Eigen::MatrixXd &M);

        static bool wbiToOrcMassMatrix(int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_orc);
        static bool wbiToOrcSegJacobian(const Eigen::MatrixXd &jac, Eigen::MatrixXd &J);
        static bool wbiToOrcBodyVector(int qdof, const Eigen::VectorXd &v_wbi, Eigen::VectorXd &v_orc);
    };

#endif

