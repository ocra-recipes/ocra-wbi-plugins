/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#include "orcWbiModel.h"

    /* static */ bool eigenDispToWbiFrame(const Eigen::Displacementd &disp, Frame &frame)
    {
        return false;
    }

    /* static */ bool wbiFrameToEigenDispd(const Frame &frame, Eigen::Displacementd &disp)
    {   
        return false;
    }

    /* static */ bool eigenRowMajorToColMajor(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::MatrixXd)
    {
        return false;
    }



