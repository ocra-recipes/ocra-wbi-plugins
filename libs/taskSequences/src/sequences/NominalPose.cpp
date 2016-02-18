
#include <taskSequences/sequences/NominalPose.h>
// NominalPoseHold
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void NominalPose::doInit(ocra::Controller& ctrl, ocra::Model& model)
    {
        tFinal = 5.0;
        tInitial = 0.0;
        t_pich_index = model.getDofIndex("torso_pitch");
        q_init = model.getJointPositions();
        nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
        getNominalPosture(model, nominal_q);

        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 20.0, 5.0, 1, q_init);
    }

    void NominalPose::doUpdate(double time, ocra::Model& state, void** args)
    {
        wocra::wOcraFullPostureTaskManager*   tmp_tmFull = dynamic_cast<wocra::wOcraFullPostureTaskManager*>(taskManagers["tmFull"]);

        Eigen::VectorXd q_current;
        if (time <= tFinal){
            q_current = (time - tInitial)/(tFinal-tInitial) * (nominal_q - q_init) + q_init;
            tmp_tmFull->setPosture(q_current);
        }
        else{
            q_current = nominal_q;
            tmp_tmFull->setPosture(q_current);
        }

        // Eigen::VectorXd taskError = tmp_tmFull->getTaskError();
        //std::cout << "Time: " << time << "[s], Posture error total: " << tmp_tmFull->getTaskErrorNorm() << std::endl;
        // std::cout << "Time: " << time << "[s], Torso Pitch error total: " << taskError(t_pich_index) << std::endl;

        /*

        for (int i = 0; i < taskError.size(); i++)
            //std::cout << "Joint " << i << ", Error : " << taskError(i) << std::endl;
            std::cout << "Joint " << i << ", Error : " << q_current(i) << std::endl;

        std::cout << std::endl << std::endl;
        */
    }
// }
