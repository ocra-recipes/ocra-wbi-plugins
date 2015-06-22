#include <gOcraController/gOcraSequenceCollection.h>
#include <../../gOcraController/include/gOcraController/ocraWbiModel.h>

#include <stdio.h>
#include <stdlib.h>
#include<string.h>

//#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
//#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"



#ifndef PI
#define PI 3.1415926
#endif

void getNominalPosture(gocra::gOcraModel &model, VectorXd &q);

// Sequence_InitialPoseHold
void Sequence_InitialPoseHold::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    FILE *infile = fopen("/home/codyco/icub/software/src/codyco-superbuild/main/ocra-wbi-plugins/modules/gOcraController/param.txt", "r");
    char keyward[256];
    char value[128];
    double kp_posture, kd_posture;

    while (fgets(keyward, sizeof(keyward), infile)){
        if (1 == sscanf(keyward, "kp_posture = %127s", value)){
            kp_posture = atof(value);
            std::cout << "kp_posture = " << kp_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kd_posture = %127s", value)){
            kd_posture = atof(value);
            std::cout << "kd_posture = " << kd_posture << std::endl;
        }
    }

    Eigen::VectorXd q_init = model.getJointPositions();
//    std::cout << "q init: " << q_init << std::endl;
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, kp_posture, kd_posture, q_init);
    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority.setZero();
    ctrl.setTaskProjectors(param_priority);
}

void Sequence_InitialPoseHold::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
}

// Sequence_NominalPoseHold
void Sequence_NominalPose::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& gmodel)
{
    model = &gmodel;
    FILE *infile = fopen("/home/codyco/icub/software/src/codyco-superbuild/main/ocra-wbi-plugins/modules/gOcraController/param.txt", "r");
    char keyward[256];
    char value[128];
    double kp_posture, kd_posture;

    while (fgets(keyward, sizeof(keyward), infile)){
        if (1 == sscanf(keyward, "kp_posture = %127s", value)){
            kp_posture = atof(value);
            std::cout << "kp_posture = " << kp_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kd_posture = %127s", value)){
            kd_posture = atof(value);
            std::cout << "kd_posture = " << kd_posture << std::endl;
        }
    }

    tFinal = 5.0;
    tInitial = 0.0;
    t_pich_index = model->getDofIndex("torso_pitch");
    q_init = model->getJointPositions();
    nominal_q = Eigen::VectorXd::Zero(model->nbInternalDofs());
    getNominalPosture(*model, nominal_q);

    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, kp_posture, kd_posture, q_init);
    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority.setZero();
    ctrl.setTaskProjectors(param_priority);

    jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/nominal/posture.txt");
    jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/nominal/dq.txt");

    counter = 0;
}

void Sequence_NominalPose::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    Eigen::VectorXd q_current;
    if (time <= tFinal){
        q_current = (time - tInitial)/(tFinal-tInitial) * (nominal_q - q_init) + q_init;
        tmFull->setPosture(q_current);
    }
    else{
        q_current = nominal_q;
        tmFull->setPosture(q_current);
    }

    jointPositionVector.push_back(model->getJointPositions());
    jointVelocityVector.push_back(model->getJointVelocities());

    if (counter == 800){
        int n = model->nbInternalDofs();
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
        for(jointPositionVectorIt = jointPositionVector.begin(); jointPositionVectorIt != jointPositionVector.end(); jointPositionVectorIt++)
        {
            q = *jointPositionVectorIt;
            for (unsigned int i=0;i<n;++i)
                jointPositionFile <<q(i)<<" ";

            jointPositionFile <<"\n";
        }
        jointPositionFile.close();

        for(jointVelocityVectorIt = jointVelocityVector.begin(); jointVelocityVectorIt != jointVelocityVector.end(); jointVelocityVectorIt++)
        {
            dq = *jointVelocityVectorIt;
            for (unsigned int i=0;i<n;++i)
                jointVelocityFile <<dq(i)<<" ";

            jointVelocityFile <<"\n";
        }
        jointVelocityFile.close();
    //             std::ostream_iterator<Eigen::VectorXd> output_iterator(postureFile, "\n");
    //             std::copy(posture.begin(), posture.end(), output_iterator);

        std::cout<<"result saved."<<std::endl;
    }

    counter += 1;

//    Eigen::VectorXd taskError = tmFull->getTaskError();

    //std::cout << "Time: " << time << "[s], Posture error total: " << tmFull->getTaskErrorNorm() << std::endl;
    // std::cout << "Time: " << time << "[s], Torso Pitch error total: " << taskError(t_pich_index) << std::endl;

    /*

    for (int i = 0; i < taskError.size(); i++)
        //std::cout << "Joint " << i << ", Error : " << taskError(i) << std::endl;
        std::cout << "Joint " << i << ", Error : " << q_current(i) << std::endl;

    std::cout << std::endl << std::endl;
    */
}

// Sequence_LeftHandReach
void Sequence_LeftHandReach::doInit(gocra::GHCJTController& controller, gocra::gOcraModel& model)
{
    FILE *infile = fopen("/home/codyco/icub/software/src/codyco-superbuild/main/ocra-wbi-plugins/modules/gOcraController/param.txt", "r");
    char keyward[256];
    char value[128];
    double kp_posture, kd_posture, kp_lh, kd_lh;

    while (fgets(keyward, sizeof(keyward), infile)){
        if (1 == sscanf(keyward, "kp_posture = %127s", value)){
            kp_posture = atof(value);
            std::cout << "kp_posture = " << kp_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kd_posture = %127s", value)){
            kd_posture = atof(value);
            std::cout << "kd_posture = " << kd_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kp_lh = %127s", value)){
            kp_lh = atof(value);
            std::cout << "kp_lh = " << kp_lh << std::endl;
        }
        if (1 == sscanf(keyward, "kd_lh = %127s", value)){
            kd_lh = atof(value);
            std::cout << "kd_lh = " << kd_lh << std::endl;
        }
    }

    ctrl = &controller;
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);

    tmFull = new gocra::gOcraFullPostureTaskManager(*ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, kp_posture, kd_posture, nominal_q);

    // Left hand cartesian task
    Eigen::Vector3d posLHandDes(-0.3, -0.1, 0.3);
    tmSegCartHandLeft = new gocra::gOcraSegCartesianTaskManager(*ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, kp_lh, kd_lh, posLHandDes);


    ctrl->setActiveTaskVector();
    nt = ctrl->getNbActiveTask();
    param_priority = Eigen::MatrixXd::Zero(nt,nt);
    param_priority(0,1)=1;//param_priority<<0,1,0,0;
    ctrl->setTaskProjectors(param_priority);
    tInitialSet=false;
//    std::cout << "Error LH: " << tmSegCartHandLeft->getTaskError().transpose() << "\n" << std::endl;
//    std::cout << "Error Pos: " << tmFull->getTaskError().transpose() << "\n" << std::endl;


}

void Sequence_LeftHandReach::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    if (!tInitialSet){
        tInitial=time;
        tInitialSet=true;
        tSwitch = tInitial + 10.0;
        switchDuration = 1.0;
        tFinal = tSwitch + switchDuration;
    }

    double t = time-tInitial;
    double coe;

//    if (t>tSwitch-0.01 && t<=tSwitch){
//        std::cout << "Error: " << tmSegCartHandLeft->getTaskError().transpose() << "\n" << std::endl;
//        std::cout << "Error Pos: " << tmFull->getTaskError().transpose() << "\n" << std::endl;
//    }
    if (t>tSwitch && t<=tFinal){
        coe = t - tSwitch;
        oneToZero = (cos(coe * M_PI/switchDuration) + 1.0)/2.0;
        zeroToOne = 1.0 - oneToZero;
        param_priority(0,1) = oneToZero;
        param_priority(1,0) = zeroToOne;
        ctrl->setTaskProjectors(param_priority);
        ctrl->doUpdateProjector();
     }
//    if (t>tFinal+10.0 && t<=tFinal+10.01){
//        std::cout << "Error: " << tmSegCartHandLeft->getTaskError().transpose() << "\n" << std::endl;
//        std::cout << "Error Pos: " << tmFull->getTaskError().transpose() << "\n" << std::endl;
//    }

}


// Sequence_ComLeftHandReach
void Sequence_ComLeftHandReach::doInit(gocra::GHCJTController& controller, gocra::gOcraModel& gmodel)
{
    FILE *infile = fopen("/home/codyco/icub/software/src/codyco-superbuild/main/ocra-wbi-plugins/modules/gOcraController/param.txt", "r");



    while (fgets(keyward, sizeof(keyward), infile)){
        if (1 == sscanf(keyward, "kp_posture = %127s", value)){
            kp_posture = atof(value);
            std::cout << "kp_posture = " << kp_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kd_posture = %127s", value)){
            kd_posture = atof(value);
            std::cout << "kd_posture = " << kd_posture << std::endl;
        }
        if (1 == sscanf(keyward, "kp_lh = %127s", value)){
            kp_lh = atof(value);
            std::cout << "kp_lh = " << kp_lh << std::endl;
        }
        if (1 == sscanf(keyward, "kd_lh = %127s", value)){
            kd_lh = atof(value);
            std::cout << "kd_lh = " << kd_lh << std::endl;
        }
        if (1 == sscanf(keyward, "kp_head = %127s", value)){
            kp_head = atof(value);
            std::cout << "kp_head = " << kp_head << std::endl;
        }
        if (1 == sscanf(keyward, "kd_head = %127s", value)){
            kd_head = atof(value);
            std::cout << "kd_head = " << kd_head << std::endl;
        }
        if (1 == sscanf(keyward, "a12 = %127s", a12)){
            a_lh_head = atof(a12);
            std::cout << "a_lh_head = " << a_lh_head << std::endl;
        }
        if (1 == sscanf(keyward, "a21 = %127s", a21)){
            a_head_lh = atof(a21);
            std::cout << "a_head_lh = " << a_head_lh << std::endl;
        }
        if (1 == sscanf(keyward, "a31 = %127s", a31)){
            a_torso_lh = atof(a31);
            std::cout << "a_torso_lh = " << a_torso_lh << std::endl;
        }
        if (1 == sscanf(keyward, "a32 = %127s", a32)){
            a_torso_head = atof(a32);
            std::cout << "a_torso_head = " << a_torso_head << std::endl;
        }
    }

    ctrl = &controller;
    model = &gmodel;
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(gmodel);
    // Full posture task
    nominal_q = Eigen::VectorXd::Zero(model->nbInternalDofs());
    getNominalPosture(*model, nominal_q);

    tmFull = new gocra::gOcraFullPostureTaskManager(*ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, kp_posture, kd_posture, nominal_q);

//    Eigen::Vector3d posHeadDes=model->getSegmentPosition(wbiModel.getSegmentIndex("head")).getTranslation();
//    std::cout<<posHeadDes.transpose()<<std::endl;

//    // Left hand cartesian task
////    Eigen::Vector3d posLHandDes=wbiModel.getSegmentPosition(wbiModel.getSegmentIndex("l_hand")).getTranslation();
////    posLHandDes(0)-=0.05;
////    posLHandDes(1)-=0.01;
////    posLHandDes(2)+=0.0;
//    Eigen::Vector3d posLHandDes(-0.3, -0.1, 0.3);

//    tmSegCartHandLeft = new gocra::gOcraSegCartesianTaskManager(*ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 64.0, 20.0, posLHandDes);

//    // CoM Task
//    Eigen::Vector3d posCoM = model.getCoMPosition();
//    tmCoM = new gocra::gOcraCoMTaskManager(*ctrl, model, "CoMTask", ocra::X, 400.0, 60.0, posCoM);//w=10

//    // Right hand cartesian task
//    Eigen::Vector3d posRHandDes=wbiModel.getSegmentPosition(wbiModel.getSegmentIndex("r_hand")).getTranslation();
//    tmSegCartHandRight = new gocra::gOcraSegCartesianTaskManager(*ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, 49.0, 20.0, posRHandDes);//w=100

//    // Partial (torso) posture task

//    Eigen::VectorXi torso_indices(3);
//    Eigen::VectorXd torsoTaskPosDes(3);
//    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
//    torsoTaskPosDes << M_PI / 18, 0, 0;
//    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(*ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 1.0, 0.5, torsoTaskPosDes);//w=5

//    // Right foot cartesian task
//    Eigen::Vector3d posRFootDes=wbiModel.getSegmentPosition(wbiModel.getSegmentIndex("r_foot")).getTranslation();
//    tmSegCartFootRight = new gocra::gOcraSegCartesianTaskManager(*ctrl, model, "rightFootCartesianTask", "r_foot", ocra::XYZ, 49.0, 20.0, posRFootDes);//w=100

//    // Left foot cartesian task
//    Eigen::Vector3d posLFootDes=wbiModel.getSegmentPosition(wbiModel.getSegmentIndex("l_foot")).getTranslation();
//    tmSegCartFootLeft = new gocra::gOcraSegCartesianTaskManager(*ctrl, model, "leftFootCartesianTask", "l_foot", ocra::XYZ, 49.0, 20.0, posLFootDes);//w=100

    ctrl->setActiveTaskVector();
    nt = ctrl->getNbActiveTask();
    param_priority = Eigen::MatrixXd::Zero(nt,nt);


//    param_priority(1,0)=1;                        param_priority(1,2)=1;
//    param_priority(2,0)=1; param_priority(2,1)=0;
//    param_priority(3,0)=1; param_priority(3,1)=1; param_priority(3,2)=1;
//    param_priority(4,0)=1; param_priority(4,1)=1; param_priority(4,2)=1;


    ctrl->setTaskProjectors(param_priority);
    tInitialSet=false;

    //plot data
    counter = 0;
    end = 3000;
    errCoM.resize(100000);
    errLH.resize(100000);
    vecT.resize(100000);



}

void Sequence_ComLeftHandReach::doUpdate(double time, gocra::gOcraModel& state, void** args)
{


    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(*model);
    int start = 400;

    if (counter==start){
        if (!tInitialSet){
            // 1: Left hand cartesian task
//                Eigen::Vector3d posLHandDes=model->getSegmentPosition(wbiModel.getSegmentIndex("l_hand")).getTranslation();
//                std::cout<<posLHandDes.transpose()<<std::endl;
            //    posLHandDes(0)-=0.05;
            //    posLHandDes(1)-=0.01;
            //    posLHandDes(2)+=0.0;
            Eigen::Vector3d posLHandDes(-0.305, -0.25, 0.0);
//            Eigen::Vector3d posLHandDes(-0.27, -0.255, 0.04);
            tmSegCartHandLeft = new gocra::gOcraSegCartesianTaskManager(*ctrl, *model, "leftHandCartesianTask", "l_hand", ocra::XYZ, kp_lh, kd_lh, posLHandDes);

            // 2: head cartesian task
            Eigen::Vector3d posHeadDes(-0.067, -0.006, 0.224);
            tmHead = new gocra::gOcraSegCartesianTaskManager(*ctrl, *model, "headCartesianTask", "head", ocra::XYZ, kp_head, kd_head, posHeadDes);
            // 2: CoM Task
//            Eigen::Vector3d posCoM = model->getCoMPosition();
//            double sqrt_kp = 20.0;
//            tmCoM = new gocra::gOcraCoMTaskManager(*ctrl, *model, "CoMTask", ocra::XY, sqrt_kp*sqrt_kp, sqrt_kp*2.0, posCoM);//w=10 400 45

            // 3: Right hand cartesian task
//            Eigen::Vector3d posRHandDes=wbiModel.getSegmentPosition(wbiModel.getSegmentIndex("r_hand")).getTranslation();
//            tmSegCartHandRight = new gocra::gOcraSegCartesianTaskManager(*ctrl, *model, "rightHandCartesianTask", "r_hand", ocra::XYZ, 25.0, 10.0, posRHandDes);//w=100

//             4: Partial (torso) posture task
            Eigen::VectorXi torso_indices(7);
            Eigen::VectorXd torsoTaskPosDes(7);
            torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw"),
                    wbiModel.getDofIndex("l_elbow"),wbiModel.getDofIndex("l_shoulder_pitch"),wbiModel.getDofIndex("l_shoulder_yaw"),wbiModel.getDofIndex("l_shoulder_roll");
            for(int i=0; i<7;++i)
            {
                torsoTaskPosDes(i) = nominal_q(torso_indices(i));
            }

//            Eigen::VectorXi torso_indices(15);
//            Eigen::VectorXd torsoTaskPosDes(15);
//            torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw"), wbiModel.getDofIndex("r_hip_roll"),wbiModel.getDofIndex("l_hip_yaw"),
//                    wbiModel.getDofIndex("l_knee"), wbiModel.getDofIndex("r_knee"), wbiModel.getDofIndex("l_ankle_roll"), wbiModel.getDofIndex("r_ankle_roll"),wbiModel.getDofIndex("l_ankle_pitch"),
//                    wbiModel.getDofIndex("r_ankle_pitch"),wbiModel.getDofIndex("l_shoulder_yaw"),wbiModel.getDofIndex("r_shoulder_pitch"), wbiModel.getDofIndex("l_elbow"), wbiModel.getDofIndex("r_elbow");
//            for(int i=0; i<15;++i)
//            {
//                torsoTaskPosDes(i) = nominal_q(torso_indices(i));
//            }



//            Eigen::VectorXi torso_indices(15);
//            Eigen::VectorXd torsoTaskPosDes(15);
//            torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw"), wbiModel.getDofIndex("r_hip_roll"), wbiModel.getDofIndex("l_hip_roll"), wbiModel.getDofIndex("l_knee"), wbiModel.getDofIndex("r_knee"),
//                    wbiModel.getDofIndex("l_ankle_roll"), wbiModel.getDofIndex("r_ankle_roll"),wbiModel.getDofIndex("l_ankle_pitch"), wbiModel.getDofIndex("r_ankle_pitch"),wbiModel.getDofIndex("l_shoulder_yaw"),wbiModel.getDofIndex("r_shoulder_pitch"), wbiModel.getDofIndex("l_elbow"), wbiModel.getDofIndex("r_elbow");
//            for(int i=0; i<15;++i)
//            {
//                torsoTaskPosDes(i) = nominal_q(torso_indices(i));
//            }

            tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(*ctrl, *model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, kp_posture, kd_posture, torsoTaskPosDes);//w=5



            ctrl->setActiveTaskVector();
            nt = ctrl->getNbActiveTask();
            param_priority = Eigen::MatrixXd::Zero(nt,nt);


            param_priority = Eigen::MatrixXd::Zero(nt,nt);
            param_priority(0,1)=1.0; param_priority(0,2)=1.0; param_priority(0,3)=0.0;// param_priority(0,4)=1; //param_priority(0,5)=1; param_priority(0,6)=1;
            param_priority(1,0)=0.0; param_priority(1,2)=a_lh_head; //param_priority(1,4)=0.1;//0.0,0.2,0.65,0.0,0.65,0.8
            param_priority(2,0)=0.0; param_priority(2,1)=a_head_lh;//1.0,0.8,0.8,0.0,0.65,0.8
            param_priority(1,3)=0.8*(param_priority(1,2));param_priority(2,3)=0.8*(param_priority(2,1));
            param_priority(3,1)=a_torso_lh;//*(1-param_priority(1,2));
            param_priority(3,2)=a_torso_head;//*(1-param_priority(2,1));//param_priority(3,1)=1; param_priority(3,2)=1;//param_priority(3,4)=0.5;//param_priority(3,5)=0.5;param_priority(3,6)=0.5;
//            param_priority(4,1)=0.9; param_priority(4,2)=1;param_priority(4,3)=0.5;//param_priority(4,5)=0.5;param_priority(4,6)=0.5;
//            param_priority(5,1)=1; param_priority(5,2)=1;param_priority(5,4)=0.5;
//            param_priority(6,1)=1; param_priority(6,2)=1;param_0priority(6,4)=0.5;



            ctrl->setTaskProjectors(param_priority);
            ctrl->doUpdateProjector();
            tInitialSet = true;
        }


    }
    else if (counter>start && counter-start <= end){
        errCoM[counter-start] = tmHead->getTaskError().norm();//tmCoM->getTaskError().norm();
        errLH[counter-start] = tmSegCartHandLeft->getTaskError().norm();
        vecT[counter-start] = (counter-start)*0.01;


        jointPositionVector.push_back(model->getJointPositions());
        jointVelocityVector.push_back(model->getJointVelocities());

        if (counter-start == end){
            if (strcmp(a12,"0.0")==0 && strcmp(a21,"1.0")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_1-0/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_1-0/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_1-0/dq.txt");
            }
            else if (strcmp(a12,"0.2")==0 && strcmp(a21,"0.8")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-2_0-8/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-2_0-8/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-2_0-8/dq.txt");
            }
            else if (strcmp(a12,"0.65")==0 && strcmp(a21,"0.8")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-8/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-8/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-8/dq.txt");
            }
            else if (strcmp(a12,"0.0")==0 && strcmp(a21,"0.0")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_0-0/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_0-0/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-0_0-0/dq.txt");
            }
            else if (strcmp(a12,"0.65")==0 && strcmp(a21,"0.65")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-6/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-6/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-6_0-6/dq.txt");
            }
            else if (strcmp(a12,"0.8")==0 && strcmp(a21,"0.8")==0){
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-8_0-8/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-8_0-8/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha_0-8_0-8/dq.txt");
            }
            else{
                resultFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha/GHCJT_motion_change_alpha.txt");
                jointPositionFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha/posture.txt");
                jointVelocityFile.open ("../../../main/ocra-wbi-plugins/modules/gOcraController/results/alpha/dq.txt");
            }

            Eigen::Vector3d posLHandDes=model->getSegmentPosition(wbiModel.getSegmentIndex("l_hand")).getTranslation();
            std::cout<<posLHandDes.transpose()<<std::endl;
            Eigen::Vector3d posHeadDes=model->getSegmentPosition(wbiModel.getSegmentIndex("head")).getTranslation();
            std::cout<<posHeadDes.transpose()<<std::endl;
            int endindex = end-1;
            for (unsigned int i=0;i<endindex;++i)
                resultFile <<errCoM[i]<<" ";
            resultFile <<errCoM[endindex]<<"\n";
            for (unsigned int i=0;i<endindex;++i)
                resultFile <<errLH[i]<<" ";
            resultFile <<errLH[endindex]<<"\n";
            for (unsigned int i=0;i<endindex;++i)
                resultFile <<vecT[i]<<" ";
             resultFile <<vecT[endindex]<<"\n";

             for (unsigned int i=0;i<endindex;++i)
                 resultFile <<vecT[i]<<" ";
              resultFile <<vecT[endindex]<<"\n";
             resultFile.close();
             std::cout<<"result saved."<<std::endl;

             int n = model->nbInternalDofs();
             Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
             Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
             for(jointPositionVectorIt = jointPositionVector.begin(); jointPositionVectorIt != jointPositionVector.end(); jointPositionVectorIt++)
             {
                 q = *jointPositionVectorIt;
                 for (unsigned int i=0;i<n;++i)
                     jointPositionFile <<q(i)<<" ";

                 jointPositionFile <<"\n";
             }
             jointPositionFile.close();

             for(jointVelocityVectorIt = jointVelocityVector.begin(); jointVelocityVectorIt != jointVelocityVector.end(); jointVelocityVectorIt++)
             {
                 dq = *jointVelocityVectorIt;
                 for (unsigned int i=0;i<n;++i)
                     jointVelocityFile <<dq(i)<<" ";

                 jointVelocityFile <<"\n";
             }
             jointVelocityFile.close();
//             std::ostream_iterator<Eigen::VectorXd> output_iterator(postureFile, "\n");
//             std::copy(posture.begin(), posture.end(), output_iterator);

             std::cout<<"result saved."<<std::endl;
         }
    }
    counter += 1;
}

// Sequence_LeftRightHandReach
void Sequence_LeftRightHandReach::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);

    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 1.0, 0.01, nominal_q);//w=0.01

    // Partial (torso) posture task

    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << M_PI / 18, 0, 0;
    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 1.0, 0.01, torsoTaskPosDes);//w=5


    // CoM Task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    tmCoM = new gocra::gOcraCoMTaskManager(ctrl, model, "CoMTask", ocra::XYZ, 49.0, 14.0, posCoM);//w=10

    // Left hand cartesian task
    Eigen::Vector3d posLHandDes(-0.3, -0.2, 0.15);
    tmSegCartHandLeft = new gocra::gOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 49.0, 14.0, posLHandDes);//w=100


    // Right hand cartesian task
    Eigen::Vector3d posRHandDes(-0.15, 0.2, -0.1);
    tmSegCartHandRight = new gocra::gOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, 49.0, 14.0, posRHandDes);//w=100

    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority<<0,1,1,1,1,
            0,0,0.9,1,1,
            0,0,0,0.9,0.9,
            0,0,0,0,0;
    ctrl.setTaskProjectors(param_priority);
}

void Sequence_LeftRightHandReach::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
}

void Sequence_CartesianTest::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);
    //double wFullPosture = 0.001;
    //double wPartialPosture = 0.01;
    //double wLeftHandTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << M_PI / 18, 0, 0;
    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, torsoTaskPosDes);

    lHandIndex = model.getSegmentIndex("l_hand");

    Eigen::Vector3d YZ_disp;
    YZ_disp << 0.0, 0.2, 0.2;


    Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);

    // start and end positions
    Eigen::Vector3d startingPos = startingDispd.getTranslation();
    desiredPos  = startingPos+ YZ_disp;


    // Left hand cartesian task
    tmLeftHandCart = new gocra::gOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp, Kd, desiredPos);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority<<0,1,1,
            0,0,1,
            0,0,0;
    ctrl.setTaskProjectors(param_priority);

}

void Sequence_CartesianTest::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    // tmLeftHandCart->setPosition(desiredPos);
    std::cout << "\n---\nDesired position: " << desiredPos.transpose() << std::endl;
    std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
    std::cout << "Error: " << tmLeftHandCart->getTaskError().transpose() << "\n" << std::endl;
}











void Sequence_PoseTest::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);
    //double wFullPosture = 0.001;
    //double wPartialPosture = 0.01;
    //double wLeftHandTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << M_PI / 18, 0, 0;
    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, torsoTaskPosDes);

    lHandIndex = model.getSegmentIndex("l_hand");

    Eigen::Vector3d YZ_disp;
    YZ_disp << 0.0, 0.2, 0.2;


    Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);
    std::cout << startingDispd << std::endl;
    // start and end positions
    Eigen::Vector3d desiredPos = startingDispd.getTranslation() + YZ_disp;
    // Eigen::Rotation3d desiredOrientation = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
    Eigen::Rotation3d desiredOrientation = startingDispd.getRotation();

    endingDispd    = Eigen::Displacementd(desiredPos, desiredOrientation);//, startingDispd.getRotation());//.inverse());

    // endingDispd    = startingDispd;
    std::cout << endingDispd << std::endl;
    // Left hand cartesian task
    tmLeftHandPose      = new gocra::gOcraSegPoseTaskManager(ctrl, model, "leftHandPoseTask", "l_hand", ocra::XYZ, Kp, Kd, endingDispd);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority<<0,1,1,
            0,0,1,
            0,0,0;
    ctrl.setTaskProjectors(param_priority);
}

void Sequence_PoseTest::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    // tmLeftHandCart->setPosition(desiredPos);
    std::cout << "\n---\nDesired pose: " << endingDispd << std::endl;
    std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex)<< std::endl;
    std::cout << "Error: " << tmLeftHandPose->getTaskError().transpose() << "\n" << std::endl;
    // tmLeftHandPose->printTaskErrors();
}















void Sequence_OrientationTest::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);
    //double wFullPosture = 0.001;
    //double wPartialPosture = 0.01;
    //double wLeftHandTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << M_PI / 18, 0, 0;
    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, Kp, Kd, torsoTaskPosDes);

    lHandIndex = model.getSegmentIndex("l_hand");

    Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);

    // start and end rotations
    startingRotd      = startingDispd.getRotation();
    endingRotd        = Eigen::Rotation3d(0.105135,   0.0828095,   0.253438,    -0.958049);// palm down //startingRotd.inverse();


    // Left hand orientation task
    tmLeftHandOrient    = new gocra::gOcraSegOrientationTaskManager(ctrl, model, "leftHandOrientationTask", "l_hand", Kp, Kd, endingRotd);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority<<0,1,1,
            0,0,1,
            0,0,0;
    ctrl.setTaskProjectors(param_priority);
}

void Sequence_OrientationTest::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    std::cout << "\n---\nStarting orientation: " << startingRotd << std::endl;
    std::cout << "Desired orientation: " << endingRotd << std::endl;
    std::cout << "Current orientation: " << state.getSegmentPosition(lHandIndex).getRotation()<< std::endl;
    std::cout << "Error: " << tmLeftHandOrient->getTaskError().transpose() << "\n" << std::endl;
    // tmLeftHandOrient->printTaskErrors();
}









/*

// Sequence_TrajectoryTrackingTest
void Sequence_TrajectoryTrackingTest::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);

    double Kp_hand = 40.0;
    double Kd_hand = 8.0 ;//* sqrt(Kp_hand);
    double wFullPosture = 0.0001;
    double wPartialPosture = 0.1;
    double wLeftHandTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0, -10.0*(M_PI / 180.0), 40.0*(M_PI / 180.0);
    // torsoTaskPosDes << 0.0, 0.0, 0.0;
    tmPartialTorso = new gocra::gOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 6., 2.0 * sqrt(6.), torsoTaskPosDes);

    // Right hand cartesian task
    Eigen::Vector3d posRHandDesDelta(0.1, 0.08, 0.15);

    Eigen::Vector3d posRHandDes = model.getSegmentPosition(model.getSegmentIndex("r_hand")).getTranslation();
    posRHandDes = posRHandDes + posRHandDesDelta;


    tmSegCartHandRight = new gocra::gOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, posRHandDes);

    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority<<0,1,1,
            0,0,1,
            0,0,0;
    ctrl.setTaskProjectors(param_priority);


    // Left hand task. Pick one of these booleans to test the different constructors.

    //Type of Trajectory //
    bool isLinInterp = false;
    bool isMinJerk = true;


    // Type of Reference
    isDisplacementd         = false;
    isRotation3d            = false;
    isCartesion             = false;
    isCartesionWaypoints    = true;

    int boolSum = isCartesion + isCartesionWaypoints + isDisplacementd + isRotation3d;
    if (boolSum>1){std::cout << "\nYou picked too many reference types." << std::endl;}
    else if (boolSum<1){std::cout << "\nYou picked too few reference types." << std::endl;}

    lHandIndex = model.getSegmentIndex("l_hand");

    Eigen::Vector3d YZ_disp;
    YZ_disp << -0.15, 0.1, 0.2;

    // start and end displacements & rotations
    Eigen::Displacementd startingDispd  = model.getSegmentPosition(lHandIndex);
    Eigen::Rotation3d startingRotd      = startingDispd.getRotation();

    endingRotd  = Eigen::Rotation3d(0.105135,   0.0828095,   0.253438,    -0.958049);
    endingDispd = Eigen::Displacementd(startingDispd.getTranslation() + YZ_disp, endingRotd);

    // start and end positions
    Eigen::Vector3d startingPos = startingDispd.getTranslation();
    desiredPos  = startingPos + YZ_disp;

    // multiple position waypoints
    Eigen::MatrixXd waypoints(3,5);
    Eigen::MatrixXd squareDisplacement(3,5);
    waypoints << startingPos, startingPos, startingPos, startingPos, startingPos;
    squareDisplacement << 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.2, 0.2, 0.0, 0.0,
                          0.0, 0.0, 0.2, 0.2, 0.0;
    waypoints += squareDisplacement;

    if (isLinInterp)
    {

        // Linear interpolation trajectory constructor tests:

        if      (isDisplacementd)       {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingDispd, endingDispd);}
        else if (isRotation3d)          {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingRotd, endingRotd);}
        else if (isCartesion)           {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingPos, desiredPos);}
        else if (isCartesionWaypoints)  {leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(waypoints);}
        else                            {std::cout << "\nGotta pick a reference type motherfucker!" << std::endl;}
    }
    else if (isMinJerk)
    {

        // Minimum jerk trajectory constructor tests:

        if      (isDisplacementd)       {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingDispd, endingDispd);}
        else if (isRotation3d)          {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingRotd, endingRotd);}
        else if (isCartesion)           {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingPos, desiredPos);}
        else if (isCartesionWaypoints)  {leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(waypoints);}
        else                            {std::cout << "\nGotta pick a reference type motherfucker!" << std::endl;}
    }
    else{std::cout << "\nGotta pick a trajectory type motherfucker!" << std::endl;}


    leftHandTrajectory->generateTrajectory(3.0); // set a 4 second duration

    if      (isDisplacementd)      {tmLeftHandPose      = new wocra::wOcraSegPoseTaskManager(ctrl, model, "leftHandPoseTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingDispd);}
    else if (isRotation3d)         {tmLeftHandOrient    = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "leftHandOrientationTask", "l_hand", Kp_hand, Kd_hand, wLeftHandTask, startingRotd);}
    else if (isCartesion)          {tmLeftHandCart      = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}
    else if (isCartesionWaypoints) {tmLeftHandCart      = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, startingPos);}


// tmSegCartHandRight = new gocra::gOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, posRHandDes);
}



void Sequence_TrajectoryTrackingTest::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    if (isDisplacementd)
    {
        Eigen::Displacementd desiredPose;
        Eigen::Twistd desiredVelocity;
        Eigen::Twistd desiredAcceleration;
        leftHandTrajectory->getDesiredValues(time, desiredPose, desiredVelocity, desiredAcceleration);

        tmLeftHandPose->setState(desiredPose, desiredVelocity, desiredAcceleration);

        // tmLeftHandPose->setState(desiredPose, desiredVelocity, Eigen::Twistd::Zero());

        std::cout << "\nFinal desired pose: " << endingDispd << std::endl;
        std::cout << "Desired pose: " << desiredPose << std::endl;
        std::cout << "Desired vel: " << desiredVelocity.transpose() << std::endl;
        std::cout << "Desired acc: " << desiredAcceleration.transpose() << std::endl;
        std::cout << "Current pose: " << state.getSegmentPosition(lHandIndex) << std::endl;
        std::cout << "Error: " << tmLeftHandPose->getTaskError().transpose() << "   norm: " << tmLeftHandPose->getTaskErrorNorm() << std::endl;

    }
    else if (isRotation3d)
    {
        Eigen::Rotation3d desiredOrientation;
        leftHandTrajectory->getDesiredValues(time, desiredOrientation);
        tmLeftHandOrient->setOrientation(desiredOrientation);

        std::cout << "\nFinal desired orientation: " << endingRotd << std::endl;
        std::cout << "Desired orientation: " << desiredOrientation << std::endl;
        std::cout << "Current orientation: " << state.getSegmentPosition(lHandIndex).getRotation() << std::endl;
        std::cout << "Error: " << tmLeftHandOrient->getTaskError().transpose() << "   norm: " << tmLeftHandOrient->getTaskErrorNorm() << std::endl;
    }
    else if (isCartesion || isCartesionWaypoints)
    {
        Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);
        // Eigen::MatrixXd H_adj = state.getSegmentPosition(lHandIndex).getRotation().inverse().adjoint();
        // H_adj*
        tmLeftHandCart->setState(desiredPosVelAcc.col(0));//,  desiredPosVelAcc.col(1), desiredPosVelAcc.col(2));

        // if(isCartesion){std::cout << "\nFinal desired position: " << desiredPos.transpose() << std::endl;}
        // std::cout << "\nDesired position: " << desiredPosVelAcc.col(0).transpose() << std::endl;
        // std::cout << "Current position: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
        // std::cout << "Error: " << tmLeftHandCart->getTaskError().transpose() << "   norm: " << tmLeftHandCart->getTaskErrorNorm() << std::endl;
    }



}
*/

void Sequence_FloatingBaseEstimationTests::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    Eigen::VectorXd q_init = model.getJointPositions();
    std::cout << "q init: " << q_init << std::endl;
    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 3.0, q_init);

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority.setZero();
    ctrl.setTaskProjectors(param_priority);
}

void Sequence_FloatingBaseEstimationTests::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
}





// Sequence_LeftRightHandReach
void Sequence_JointTest::doInit(gocra::GHCJTController& ctrl, gocra::gOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
    // Full posture task
    nDoF = model.nbInternalDofs();
    // Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(nDoF);
    // getNominalPosture(model, nominal_q);


    q_init = model.getJointPositions();

    q_des = q_init;

    jointMin = model.getJointLowerLimits();

    jointMax = model.getJointUpperLimits();

    tmFull = new gocra::gOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 2.0*sqrt(20), q_init);
    for (int i=0; i<nDoF; i++){
        jointNames[i] = wbiModel.getJointName(i);
    }
    taskErr = 0.0;
    jIndex = 0;
    goToMin = true;
    goToMax = false;
    counter = 401;

    ctrl.setActiveTaskVector();
    int nt = ctrl.getNbActiveTask();
    Eigen::MatrixXd param_priority(nt,nt);
    param_priority.setZero();
    ctrl.setTaskProjectors(param_priority);

}

void Sequence_JointTest::doUpdate(double time, gocra::gOcraModel& state, void** args)
{
    Eigen::VectorXd taskErrorVector = tmFull->getTaskError();
    // std::cout << taskErrorVector.transpose() << std::endl;
    taskErr = abs(taskErrorVector(jIndex));
    // std::cout << taskErr << std::endl;




    if (( counter>=400) && jIndex<nDoF){
        if ((goToMin == true) && (goToMax==false)){

            q_des(jIndex) = jointMin(jIndex);
            tmFull->setPosture(q_des);
            goToMin = false;
            goToMax = true;
            std::cout << "\nJoint: " << jointNames[jIndex] << "-> moving to lower limit, " << jointMin(jIndex) << std::endl;
        }

        else if ((goToMin == false )&& (goToMax==true)){
            q_des(jIndex) = jointMax(jIndex);
            tmFull->setPosture(q_des);
            goToMax = false;
            std::cout << "\nJoint: " << jointNames[jIndex] << "-> moving to upper limit, " << jointMax(jIndex) << std::endl;

        }
        else if ((goToMin == false) && (goToMax==false)){
            q_des = q_init;
            tmFull->setPosture(q_des);
            jIndex++;
            goToMin = true;

        }
        counter = 0;
    }

    counter ++ ;


}


void getNominalPosture(gocra::gOcraModel& model, VectorXd &q)
{
    q[model.getDofIndex("torso_pitch")] = M_PI / 18;
    q[model.getDofIndex("r_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("l_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("r_hip_pitch")] = M_PI / 8;
    q[model.getDofIndex("l_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("r_hip_roll")] = M_PI / 18;
    q[model.getDofIndex("l_knee")] = -M_PI / 6;
    q[model.getDofIndex("r_knee")] = -M_PI / 6;
}
