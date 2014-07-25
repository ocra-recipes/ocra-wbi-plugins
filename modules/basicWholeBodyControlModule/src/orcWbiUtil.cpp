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



