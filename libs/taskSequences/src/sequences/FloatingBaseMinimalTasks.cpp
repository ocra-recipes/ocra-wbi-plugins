
#include <taskSequences/sequences/FloatingBaseMinimalTasks.h>
// FloatingBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FloatingBaseMinimalTasks::doInit(wocra::wOcraController& c, wocra::wOcraModel& m)
    {
        ctrl = &c;
        model = &m;

        // Initialise full posture task
        // Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
        Eigen::VectorXd q_full = model->getJointPositions();


        // q_full[model.getDofIndex("l_elbow")]      = PI/8.0;
        // q_full[model.getDofIndex("r_elbow")]      = PI/8.0;
        // q_full[model.getDofIndex("l_knee")]             = -0.05;
        // q_full[model.getDofIndex("r_knee")]             = -0.05;
        // q_full[model.getDofIndex("l_ankle_pitch")]      = -0.05;
        // q_full[model.getDofIndex("r_ankle_pitch")]      = -0.05;
        // q_full[model.getDofIndex("l_shoulder_roll")]    = PI/8.0;
        // q_full[model.getDofIndex("r_shoulder_roll")]    = PI/8.0;


        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(*ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, 1.0, 1*sqrt(1.0), 0.001, q_full);

        // Initialise partial posture task
        Eigen::VectorXd q_partial(3);
        Eigen::VectorXi q_partial_index(3);
        q_partial_index<<model->getDofIndex("torso_pitch"),model->getDofIndex("torso_roll"),model->getDofIndex("torso_yaw");
        q_partial<<q_full[model->getDofIndex("torso_pitch")],q_full[model->getDofIndex("torso_roll")],q_full[model->getDofIndex("torso_yaw")];

        taskManagers["tmPartial"] = new wocra::wOcraPartialPostureTaskManager(*ctrl, *model, "partialPostureTask", ocra::FullState::INTERNAL, q_partial_index, 10.0, 1.0*sqrt(10.0), 1.0, q_partial);

        // Initialise com task
        Eigen::Vector3d desiredCoMPosition;
        desiredCoMPosition = model->getCoMPosition();
        taskManagers["tmCoM"] = new wocra::wOcraCoMTaskManager(*ctrl, *model, "CoMTask", 10.0, 1*sqrt(10.0), 10.0, desiredCoMPosition);
        tmCoM = taskManagers["tmCoM"];

        // Initialise waist pose
        Eigen::Vector3d desiredWaistPosition, XYZdisp;
        desiredWaistPosition = model->getSegmentPosition(model->getSegmentIndex("torso")).getTranslation();
        XYZdisp << 0.0, 0.0, 0.0;
        desiredWaistPosition = desiredWaistPosition + XYZdisp;
        taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "waistPoseTask", "torso", ocra::XYZ, 1.0, 0.1*sqrt(1.0), 1.0, desiredWaistPosition);

        // Initialise head pose
        Eigen::Vector3d desiredHeadPosition, headdisp;
        desiredHeadPosition = model->getSegmentPosition(model->getSegmentIndex("head")).getTranslation();
        headdisp << 0.0, 0.0, 0.0;
        desiredHeadPosition = desiredHeadPosition + headdisp;
        taskManagers["tmSegPoseHead"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "headPoseTask", "head", ocra::XYZ, 1.0, 0.1*sqrt(1.0), 1.0, desiredHeadPosition);


        // Left hand cartesian task
        Eigen::Vector3d posLHandDes, handdisp;
        posLHandDes = model->getSegmentPosition(model->getSegmentIndex("l_hand")).getTranslation();
        handdisp<<0.03, -0.01, 0.03;
        posLHandDes = posLHandDes + handdisp;
        taskManagers["tmSegCartHandLeft"] = new wocra::wOcraSegCartesianTaskManager(*ctrl, *model, "leftHandCartesianTask", "l_hand", ocra::XYZ, 1.0, 1*sqrt(1.0), 1.0, posLHandDes);



        double mu_sys = 0.5;
        double margin = 0.0;
        double sqrt2on2 = sqrt(2.0)/2.0;

        Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
        Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);


        // Initialise left foot contacts
        Eigen::Displacementd LFContacts[4];
        LFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown);
        LFContacts[1] = Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown);
        LFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown);
        LFContacts[3] = Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown);
        taskManagers["tmFootContactLeft"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "leftFootContactTask", "l_sole", LFContacts, 4, mu_sys, margin);

        // Initailise right foot contacts
        Eigen::Displacementd RFContacts[4];
        RFContacts[0] = Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown);
        RFContacts[1] = Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown);
        RFContacts[2] = Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown);
        RFContacts[3] = Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown);
        taskManagers["tmFootContactRight"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "RightFootContactTask", "r_sole", RFContacts, 4, mu_sys, margin);

    }

    void FloatingBaseMinimalTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
        std::cout<<model->getCoMPosition()[0,0]<<std::endl;
        if (time==0.15)
        {

            Eigen::Vector3d desiredCoMPosition, CoMdisp;
            desiredCoMPosition = model->getCoMPosition();
            CoMdisp << 0.01, 0.0, 0.0;
            desiredCoMPosition = desiredCoMPosition + CoMdisp;

            tmCoM->setState(desiredCoMPosition);

        }
    }

// }
