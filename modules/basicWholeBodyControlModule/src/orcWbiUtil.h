/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#ifndef ORCWBIUTIL_H
#define ORCWBIUTIL_H

    /* Conversions between types/conventions used in Eigen and WBI */
    class orcWbiConversions
    {
    public:
        static bool eigenDispToWbiFrame(const Eigen::Displacementd &disp, Frame &frame);
        static bool wbiFrameToEigenDispd(const Frame &frame, Eigen::Displacementd &disp);
        static bool eigenRowMajorToColMajor(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::MatrixXd);
    };

#endif

