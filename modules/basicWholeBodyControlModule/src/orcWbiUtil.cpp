/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#include <iostream>

#include "orcWbiUtil.h"

#define DIM_T 3
#define DIM_R 3

    /* static */ bool orcWbiConversions::eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame)
    {
        return false;
    }

    /* static */ bool orcWbiConversions::wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp)
    {   
        return false;
    }

    /* static */ bool orcWbiConversions::eigenRowMajorToColMajor(const MatrixXdRm &M_rm, Eigen::MatrixXd &M)
    {
        if((M_rm.cols() != M.cols()) || (M_rm.rows() != M.rows()))
        {
            std::cout<<"ERROR: Sizes of row major matrix and col major matrix are inconsistent"<<std::endl;
            return false;
        }

        for(unsigned int i = 0; i < M_rm.rows(); i++)
            for(unsigned int j = 0; j < M_rm.cols(); j++)
                M(i,j) = M_rm(i,j);
            
        return true;
    }

    /* static */ bool orcWbiConversions::massMatrixWbiToOrc(int dof, int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_orc)
    {
        Eigen::MatrixXd m11(qdof, qdof);
        Eigen::MatrixXd m12(qdof, DIM_T);
        Eigen::MatrixXd m13(qdof, DIM_R);
        Eigen::MatrixXd m21(DIM_T, qdof);
        Eigen::MatrixXd m22(DIM_T, DIM_T);
        Eigen::MatrixXd m23(DIM_T, DIM_R);
        Eigen::MatrixXd m31(DIM_R, qdof);
        Eigen::MatrixXd m32(DIM_R, DIM_T);
        Eigen::MatrixXd m33(DIM_R, DIM_R);

        /*
            Changes matrix from
         */
        m11 = M_wbi.block(0, 0, qdof, qdof);
        m12 = M_wbi.block(0, qdof, qdof, DIM_T);
        m13 = M_wbi.block(0, qdof+DIM_T, qdof, DIM_R);
        m21 = M_wbi.block(qdof, 0, DIM_T, qdof);
        m22 = M_wbi.block(qdof, qdof, DIM_T, DIM_T);
        m23 = M_wbi.block(qdof, qdof+DIM_T, DIM_T, DIM_R);
        m31 = M_wbi.block(qdof+DIM_T, 0, DIM_T, qdof);
        m32 = M_wbi.block(qdof+DIM_T, qdof, DIM_R, DIM_R);
        m33 = M_wbi.block(qdof+DIM_T, qdof+DIM_R, DIM_R, DIM_R);

        M_orc << m22, m21, m23,
                m12, m11, m13,
                m32, m31, m33;

        return true;
    }

