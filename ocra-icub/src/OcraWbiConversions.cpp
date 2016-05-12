/*! \file       OcraWbiConversions.cpp
 *  \brief      A utility class full of static conversion functions.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ocra-icub/OcraWbiConversions.h>

using namespace ocra_icub;

/* static */ bool OcraWbiConversions::eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame)
    {
		double p_wbi[3] = {disp.x(), disp.y(), disp.z()};
		frame = wbi::Frame(wbi::Rotation::quaternion(disp.qx(), disp.qy(), disp.qz(), disp.qw()), p_wbi);
		return true;
    }

/* static */ bool OcraWbiConversions::wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp)
    {
		double qx, qy, qz, qw;
		frame.R.getQuaternion(qx, qy, qz, qw);
		disp  = Eigen::Displacementd(frame.p[0], frame.p[1], frame.p[2], qw, qx, qy, qz);
		return true;
    }

/* static */ bool OcraWbiConversions::wbiToOcraTwistVector(Eigen::Twistd &t_wbi, Eigen::Twistd &t_ocra)
    {
        Eigen::Vector3d ocrat = t_wbi.head(DIM_TRANSLATION);
        Eigen::Vector3d ocrar = t_wbi.tail(DIM_ROTATION);

        t_ocra << ocrar,
                ocrat;

        return true;
    }

/* static */ bool OcraWbiConversions::ocraToWbiTwistVector(Eigen::Twistd &t_ocra, Eigen::Twistd &t_wbi)
    {
        Eigen::Vector3d ocrar = t_ocra.head(DIM_TRANSLATION);
        Eigen::Vector3d ocrat = t_ocra.tail(DIM_ROTATION);

        t_wbi << ocrat,
                ocrar;

        return true;
    }

/* static */ bool OcraWbiConversions::eigenRowMajorToColMajor(const MatrixXdRm &M_rm, Eigen::MatrixXd &M)
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

    // ONLY USE WHEN MASS MATRIX CONTAINS FREE BASE TRANSLATION AND ROTATION
    //      Since the translation and rotation are switched
    //      WBI order [T R Q]
    //      ocra order [R T Q]
/* static */ bool OcraWbiConversions::wbiToOcraMassMatrix(int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_ocra)
    {
        int dof = qdof + DIM_TRANSLATION + DIM_ROTATION;
        if(dof != M_wbi.cols() || dof != M_wbi.rows() || dof != M_ocra.rows() || dof != M_ocra.cols())
        {
            std::cout<<"ERROR: Input and output matrices - Is the model free root?" <<std::endl;
            return false;
        }
        int dim_t = DIM_TRANSLATION;
        int dim_r = DIM_ROTATION;
        // ORIGINAL MATRIX BLOCKS
        Eigen::MatrixXd m11(dim_t, dim_t);
        m11 = M_wbi.block(0, 0, dim_t, dim_t);

        Eigen::MatrixXd m12(dim_t, dim_r);
        m12 = M_wbi.block(0, dim_t, dim_t, dim_r);

        Eigen::MatrixXd m13(dim_t, qdof);
        m13 = M_wbi.block(0, dim_t+dim_r, dim_t, qdof);

        Eigen::MatrixXd m21(dim_r, dim_t);
        m21 = M_wbi.block(dim_t, 0, dim_r, dim_t);

        Eigen::MatrixXd m22(dim_r, dim_r);
        m22 = M_wbi.block(dim_t, dim_t, dim_r, dim_r);

        Eigen::MatrixXd m23(dim_r, qdof);
        m23 = M_wbi.block(dim_t, dim_t+dim_r, dim_r, qdof);

        Eigen::MatrixXd m31(qdof, dim_t);
        m31 = M_wbi.block(dim_t+dim_r, 0, qdof, dim_t);

        Eigen::MatrixXd m32(qdof, dim_r);
        m32 = M_wbi.block(dim_t+dim_r, dim_t, qdof, dim_r);

        Eigen::MatrixXd m33(qdof, qdof);
        m33 = M_wbi.block(dim_t+dim_r, dim_t+dim_r, qdof, qdof);

        M_ocra << m22, m21, m23,
                m12, m11, m13,
                m32, m31, m33;

        return true;
    }


/* static */ bool OcraWbiConversions::wbiToOcraSegJacobian(const Eigen::MatrixXd &jac, Eigen::MatrixXd &J)
    {
        int dof = DIM_TRANSLATION + DIM_ROTATION;
        if(dof != jac.rows() || dof != J.rows()||jac.cols() != J.cols())
        {
            std::cout<<"ERROR: Input and output matrices dimensions should be the same" <<std::endl;
            return false;
        }

        // FOR FULL n+6 Jacobian ONLY
        Eigen::MatrixXd jac5,jac6;
        Eigen::Matrix3d jac1,jac2,jac3,jac4;
        jac5.resize(3,jac.cols()-6);
        jac6.resize(3,jac.cols()-6);


        jac1 = jac.topLeftCorner(3,3);
        jac2 = jac.block<3,3>(0,3);
        jac3 = jac.bottomLeftCorner(3,3);
        jac4 = jac.block<3,3>(3,3);
        jac5 = jac.topRightCorner(3,jac.cols()-6);
        jac6 = jac.bottomRightCorner(3,jac.cols()-6);

        J.topLeftCorner(3,3) = jac4;
        J.block<3,3>(0,3) = jac3;
        J.bottomLeftCorner(3,3) = jac2;
        J.block<3,3>(3,3) = jac1;
        J.topRightCorner(3,jac.cols()-6) = jac6;
        J.bottomRightCorner(3,jac.cols()-6) = jac5;

        return true;
    }


/* static */ bool OcraWbiConversions::wbiToOcraCoMJacobian(const Eigen::MatrixXd &jac, Eigen::Matrix<double,3,Eigen::Dynamic> &J)
    {

        if(DIM_TRANSLATION != jac.rows() || jac.cols() != J.cols())
        {
            std::cout<<"ERROR: Input and output matrices dimensions should be the same" <<std::endl;
            return false;
        }
        Eigen::MatrixXd jac3;
        Eigen::Matrix3d jac1,jac2;
        jac3.resize(3,jac.cols()-6);

        jac1 = jac.leftCols(3);
        jac2 = jac.block<3,3>(0,3);
        jac3 = jac.rightCols(jac.cols()-6);
        J.leftCols(3) = jac2;
        J.block<3,3>(0,3) = jac1;
        J.rightCols(jac.cols()-6) = jac3;


        return true;
    }


    // ONLY USE WHEN VECTORS CONTAINS FREE BASE TRANSLATION AND ROTATION
    //      Since the translation and rotation are switched
    //      WBI order [T R Q]
    //      ocra order [R T Q]
    /* static */ bool OcraWbiConversions::wbiToOcraBodyVector(int qdof, const Eigen::VectorXd &v_wbi, Eigen::VectorXd &v_ocra)
    {
        int dof = qdof + DIM_TRANSLATION + DIM_ROTATION;
        if(dof != v_wbi.size() || dof != v_ocra.size())
        {
            std::cout<<"ERROR: Input and output matrices - Is the model free root?" <<std::endl;
            return false;
        }

        Eigen::VectorXd t(DIM_TRANSLATION);
        Eigen::VectorXd r(DIM_ROTATION);
        Eigen::VectorXd q(qdof);

        t = v_wbi.segment(0, DIM_TRANSLATION);
        r = v_wbi.segment(DIM_TRANSLATION, DIM_ROTATION);
        q = v_wbi.segment(DIM_TRANSLATION+DIM_ROTATION, qdof);

        v_ocra << r,
                t,
                q;
    }


    //---------------------------------------------------------
        bool OcraWbiConversions::eigenToYarpVector(const Eigen::VectorXd &eigenVector, yarp::sig::Vector &yarpVector)
        {
            if(eigenVector.size() == 0)
            {
                std::cout<<"ERROR: input vector is empty (eigenToYarpVector)"<<std::endl;
                return false;
            }

            //resize and fill eigen vector with yarp vector elements
            yarpVector.resize(eigenVector.size());
            for(unsigned int i=0; i <eigenVector.size(); i++)
                    yarpVector(i) = eigenVector(i);

            return true;
        }
