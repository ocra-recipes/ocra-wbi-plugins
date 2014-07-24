/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#include "orcWbiUtil.h"

    /* static */ bool orcWbiConversions::eigenDispToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame)
    {
        return false;
    }

    /* static */ bool orcWbiConversions::wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp)
    {   
        return false;
    }

    /* static */ bool orcWbiConversions::eigenRowMajorToColMajor(MatrixXdRm &M_rm, Eigen::MatrixXd &M)
    {
        return false;
    }



