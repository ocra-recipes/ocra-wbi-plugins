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

    // ONLY USE WHEN MASS MATRIX CONTAINS FREE BASE TRANSLATION AND ROTATION
    //      Since the translation and rotation are switched
    //      WBI order [T R Q]
    //      ORC order [R T Q]
    /* static */ bool orcWbiConversions::wbiToOrcMassMatrix(int qdof, const Eigen::MatrixXd &M_wbi, Eigen::MatrixXd &M_orc)
    {
        int dof = qdof + DIM_T + DIM_R;
        if(dof != M_wbi.cols() || dof != M_wbi.rows() || dof != M_orc.rows() || dof != M_orc.cols())
        {
            std::cout<<"ERROR: Input and output matrices - Is the model free root?" <<std::endl;
            return false;
        }

        // ORIGINAL MATRIX BLOCKS
        Eigen::MatrixXd m11(DIM_T, DIM_T);
        m11 = M_wbi.block(0, 0, DIM_T, DIM_T);

        Eigen::MatrixXd m12(DIM_T, DIM_R);
        m12 = M_wbi.block(0, DIM_T, DIM_T, DIM_R);

        Eigen::MatrixXd m13(DIM_T, qdof);
        m13 = M_wbi.block(0, DIM_T+DIM_R, DIM_T, qdof);

        Eigen::MatrixXd m21(DIM_R, DIM_T);
        m21 = M_wbi.block(DIM_T, 0, DIM_R, DIM_T);

        Eigen::MatrixXd m22(DIM_R, DIM_R);
        m22 = M_wbi.block(DIM_T, DIM_T, DIM_R, DIM_R);

        Eigen::MatrixXd m23(DIM_R, qdof);
        m23 = M_wbi.block(DIM_T, DIM_T+DIM_R, DIM_R, qdof);

        Eigen::MatrixXd m31(qdof, DIM_T);
        m31 = M_wbi.block(DIM_T+DIM_R, 0, qdof, DIM_T);

        Eigen::MatrixXd m32(qdof, DIM_R);
        m32 = M_wbi.block(DIM_T+DIM_R, DIM_T, qdof, DIM_R);

        Eigen::MatrixXd m33(qdof, qdof);
        m33 = M_wbi.block(DIM_T+DIM_R, DIM_T+DIM_R, qdof, qdof);

        M_orc << m22, m21, m23,
                m12, m11, m13,
                m32, m31, m33;

        return true;
    }

    /* static */ bool orcWbiConversions::wbiToOrcSegJacobian(const Eigen::MatrixXd &jac, Eigen::MatrixXd &J)
    {
//    Eigen::MatrixXd jac_cm,jac_bottom,jac_top,jac_rt,jac_rr;
//    eigenRowMajorToColMajor(jac, jac_cm);
//    jac_tmp.resize(3,jac_cm.cols());
//    jac_top = owm_pimpl->segJacobian_cm[index].bottomRows(3);
//    jac_bottom = owm_pimpl->segJacobian_cm[index].topRows(3);
//    owm_pimpl->segJacobian[index].topRows(3) = jac_top;
//    owm_pimpl->segJacobian[index].bottomRows(3) = jac_bottom;
//    jac_rt = owm_pimpl->segJacobian[index].leftCols(3);
//    jac_rr = owm_pimpl->segJacobian[index].block<owm_pimpl->segJacobian[index].rows(),3>(0,3);

//    owm_pimpl->segJacobian_cm = owm_pimpl->segJacobian;
    }


    // ONLY USE WHEN VECTORS CONTAINS FREE BASE TRANSLATION AND ROTATION
    //      Since the translation and rotation are switched
    //      WBI order [T R Q]
    //      ORC order [R T Q]
    /* static */ bool orcWbiConversions::wbiToOrcBodyVector(int qdof, const Eigen::VectorXd &v_wbi, Eigen::VectorXd &v_orc)
    {
        int dof = qdof + DIM_T + DIM_R;
        if(dof != v_wbi.size() || dof != v_orc.cols())
        {
            std::cout<<"ERROR: Input and output matrices - Is the model free root?" <<std::endl;
            return false;
        }
        
        Eigen::VectorXd t(DIM_T);
        Eigen::VectorXd r(DIM_R);
        Eigen::VectorXd q(qdof);

        t = v_wbi.segment(0, DIM_T);
        r = v_wbi.segment(DIM_T, DIM_R);
        q = v_wbi.segment(DIM_T+DIM_R, qdof);

        v_orc << r,
                t,
                q;
    }
