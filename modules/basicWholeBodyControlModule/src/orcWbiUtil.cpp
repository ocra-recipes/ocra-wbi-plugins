/*
* Copyright (C) 2014 ISIR, UPMC
* Author: Darwin Lau, Mingxing Liu
* email: 
*
* Utility functions for working with ORC/Eigen and Whole Body Interface
*/

#include <iostream>

#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/sig/Matrix.h>
#include <map>
#include <vector>
#include "orcWbiUtil.h"

#define DIM_T 3
#define DIM_R 3

    /* static */ bool orcWbiConversions::eigenDispdToWbiFrame(const Eigen::Displacementd &disp, wbi::Frame &frame)
    {
		double p_wbi[3];
		Eigen::Vector3d p;
		p = disp.getTranslation();
		
		p_wbi[0] = p[0];
		p_wbi[1] = p[1];
		p_wbi[2] = p[2];
		
		double _x, _y, _z, _w;
		_x = disp.getRotation().x();
		_y = disp.getRotation().y();
		_z = disp.getRotation().z();
		_w = disp.getRotation().w();
				
		wbi::Rotation R;	R.quaternion(_x, _y, _z, _w);	
		wbi::Frame _frame(R, p_wbi);
		frame = _frame;
		
		// Check if y-translation value is the same between the two data containers
		if (frame.p[1]!=p(1,0)){
			std::cerr << "Displacementd could not be converted to Wbi Frame \n";
			return false;
		}
		
		return true;
    }

    /* static */ bool orcWbiConversions::wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp)
    {   
		double _x, _y, _z, _w;
		frame.R.getQuaternion(_x, _y, _z, _w);
		
		double x, y, z;
		x = frame.p[0];
		y = frame.p[1];
		z = frame.p[2];
		
		Eigen::Vector3d trans;
		trans << x, y, z;
		
		Eigen::Displacementd _disp(x,y,z,_w,_x,_y,_z);
		disp = _disp;
		
		// Check if y-translation value is the same between the two data containers
		if (disp.getTranslation()(1,0)!=y){
			std::cerr << "Wbi Frame could not be converted to Displacementd\n";
			return false;
		}
		
		return true;
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

    /* static */ bool orcWbiConversions::wbiToOrcSegJacobian(const Eigen::MatrixXd &jac, Eigen::Matrix<double,6,Eigen::Dynamic> &J)
    {
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

    /* static */ bool orcWbiConversions::wbiToOrcCoMJacobian(const Eigen::MatrixXd &jac, Eigen::Matrix<double,3,Eigen::Dynamic> &J)
    {
        Eigen::MatrixXd jac3;
        Eigen::Matrix3d jac1,jac2;
        jac3.resize(3,jac.cols()-6);

        jac1 = jac.topLeftCorner(3,3);
        jac2 = jac.block<3,3>(0,3);
        jac3 = jac.topRightCorner(3,jac.cols()-6);
        J.topLeftCorner(3,3) = jac2;
        J.block<3,3>(0,3) = jac1;
        J.topRightCorner(3,jac.cols()-6) = jac3;


        return true;
    }
