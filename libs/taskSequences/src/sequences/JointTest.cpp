#include <taskSequences/sequences/JointTest.h>
// JointTest
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void JointTest::doInit(ocra::Controller& ctrl, ocra::Model& model)
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

        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 2.0*sqrt(20), 1.0, q_init);
        for (int i=0; i<nDoF; i++){
            jointNames[i] = wbiModel.getJointName(i);
        }
        taskErr = 0.0;
        jIndex = 0;
        goToMin = true;
        goToMax = false;
        counter = 401;



    }

    void JointTest::doUpdate(double time, ocra::Model& state, void** args)
    {

        wocra::wOcraFullPostureTaskManager*   tmp_tmFull = dynamic_cast<wocra::wOcraFullPostureTaskManager*>(taskManagers["tmFull"]);

        Eigen::VectorXd taskErrorVector = tmp_tmFull->getTaskError();
        // std::cout << taskErrorVector.transpose() << std::endl;
        taskErr = std::abs(taskErrorVector(jIndex));
        // std::cout << taskErr << std::endl;




        if (( counter>=400) && jIndex<nDoF){
            if ((goToMin == true) && (goToMax==false)){

                q_des(jIndex) = jointMin(jIndex);
                tmp_tmFull->setPosture(q_des);
                goToMin = false;
                goToMax = true;
                std::cout << "\nJoint: " << jointNames[jIndex] << "-> moving to lower limit, " << jointMin(jIndex) << std::endl;
            }

            else if ((goToMin == false )&& (goToMax==true)){
                q_des(jIndex) = jointMax(jIndex);
                tmp_tmFull->setPosture(q_des);
                goToMax = false;
                std::cout << "\nJoint: " << jointNames[jIndex] << "-> moving to upper limit, " << jointMax(jIndex) << std::endl;

            }
            else if ((goToMin == false) && (goToMax==false)){
                q_des = q_init;
                tmp_tmFull->setPosture(q_des);
                jIndex++;
                goToMin = true;

            }
            counter = 0;
        }

        counter ++ ;


    }
// }
