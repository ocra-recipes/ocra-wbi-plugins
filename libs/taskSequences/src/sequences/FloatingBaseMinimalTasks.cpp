
#include <taskSequences/sequences/FloatingBaseMinimalTasks.h>
// FloatingBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FloatingBaseMinimalTasks::doInit(wocra::wOcraController& c, wocra::wOcraModel& m)
    {
        ctrl = &c;
        model = &m;
        bool usesYARP = true;

        // Initialise full posture task
        Eigen::VectorXd q_full = model->getJointPositions();
        q_full[model->getDofIndex("l_knee")] = -PI / 12;
        q_full[model->getDofIndex("r_knee")] = -PI / 12;
        taskManagers["fullPostureTask"] = new wocra::wOcraFullPostureTaskManager(*ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, 10.0, 1*sqrt(10.0), 0.01, q_full, usesYARP);


        // Initialise partial posture task
        Eigen::VectorXd q_partial(3);
        Eigen::VectorXi q_partial_index(3);
        q_partial_index<<model->getDofIndex("torso_pitch"),model->getDofIndex("torso_roll"),model->getDofIndex("torso_yaw");
        q_partial<<q_full[model->getDofIndex("torso_pitch")],q_full[model->getDofIndex("torso_roll")],q_full[model->getDofIndex("torso_yaw")];

        taskManagers["torsoPostureTask"] = new wocra::wOcraPartialPostureTaskManager(*ctrl, *model, "torsoPostureTask", ocra::FullState::INTERNAL, q_partial_index, 10.0, 2.0*sqrt(10.0), 1.0, q_partial, usesYARP);

        // Initialise com task
        Eigen::Vector3d desiredCoMPosition;
        desiredCoMPosition = model->getCoMPosition();
        taskManagers["CoMTask"] = new wocra::wOcraCoMTaskManager(*ctrl, *model, "CoMTask", 10.0, 2*sqrt(10.0), 10.0, desiredCoMPosition, usesYARP);
        tmCoM = dynamic_cast<wocra::wOcraCoMTaskManager*>(taskManagers["CoMTask"]);

        // Initialise waist pose
        Eigen::Vector3d desiredWaistPosition, XYZdisp;
        desiredWaistPosition = model->getSegmentPosition(model->getSegmentIndex("torso")).getTranslation();
        XYZdisp << 0.0, 0.0, 0.0;
        desiredWaistPosition = desiredWaistPosition + XYZdisp;
        taskManagers["waistPoseTask"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "waistPoseTask", "torso", ocra::XYZ, 1.0, 2*sqrt(1.0), 1.0, desiredWaistPosition, usesYARP);

        // Initialise head pose
        Eigen::Vector3d desiredHeadPosition, headdisp;
        desiredHeadPosition = model->getSegmentPosition(model->getSegmentIndex("head")).getTranslation();
        headdisp << 0.0, 0.0, 0.0;
        desiredHeadPosition = desiredHeadPosition + headdisp;
        taskManagers["headPoseTask"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "headPoseTask", "head", ocra::XYZ, 1.0, 2*sqrt(1.0), 1.0, desiredHeadPosition, usesYARP);


        // Left hand cartesian task
        Eigen::Vector3d posLHandDes, handdisp;
        posLHandDes = model->getSegmentPosition(model->getSegmentIndex("l_hand")).getTranslation();
        handdisp<< 0.1, 0.1, 0.1;
        posLHandDes = posLHandDes + handdisp;
        taskManagers["leftHandCartesianTask"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 20.0, 2*sqrt(20.0), 1.0, posLHandDes, usesYARP);



        double mu_sys = 0.5;
        double margin = 0.0;
        double sqrt2on2 = sqrt(2.0)/2.0;

        Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
        Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);


        // Initialise left foot contacts

        std::vector<Eigen::Displacementd> LFContacts;
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown));
        taskManagers["leftFootContactTask"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "leftFootContactTask", "l_sole", LFContacts, mu_sys, margin);

        // Initailise right foot contacts
        std::vector<Eigen::Displacementd> RFContacts;
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown));
        taskManagers["rightFootContactTask"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "rightFootContactTask", "r_sole", RFContacts, mu_sys, margin);

    }

    void FloatingBaseMinimalTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
        if (time==0.15)
        {

            Eigen::Vector3d desiredCoMPosition, CoMdisp;
            desiredCoMPosition = model->getCoMPosition();
            CoMdisp << 0.0, 0.0, -0.05;
            desiredCoMPosition = desiredCoMPosition + CoMdisp;

            tmCoM->setState(desiredCoMPosition);

        }
    }

// }
