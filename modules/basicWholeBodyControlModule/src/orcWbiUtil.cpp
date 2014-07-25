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
		// Need to add error checking...
        return false;
    }

    /* static */ bool orcWbiConversions::wbiFrameToEigenDispd(const wbi::Frame &frame, Eigen::Displacementd &disp)
    {   
		double _x, _y, _z, _w;
		frame.R.getQuaternion(_x, _y, _z, _w);
		//Eigen::Rotation3d Rot;
		//Rot << _x, _y, _z, _w;
		
		double x, y, z;
		x = frame.p[0];
		y = frame.p[1];
		z = frame.p[2];
		Eigen::Vector3d trans;
		trans << x, y, z;
		Eigen::Displacementd _disp(x,y,z,_w,_x,_y,_z);
		disp = _disp;
		// Need to add error checking...
        return false;
    }

    /* static */ bool orcWbiConversions::eigenRowMajorToColMajor(MatrixXdRm &M_rm, Eigen::MatrixXd &M)
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
