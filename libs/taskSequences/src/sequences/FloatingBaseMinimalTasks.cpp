
#include <taskSequences/sequences/FloatingBaseMinimalTasks.h>
// FloatingBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
    void FloatingBaseMinimalTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
    {
        // Initialise full posture task
        // Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
        Eigen::VectorXd q_full = model.getJointPositions();


        // q_full[model.getDofIndex("l_elbow")]      = PI/8.0;
        // q_full[model.getDofIndex("r_elbow")]      = PI/8.0;
        // q_full[model.getDofIndex("l_knee")]             = -0.05;
        // q_full[model.getDofIndex("r_knee")]             = -0.05;
        // q_full[model.getDofIndex("l_ankle_pitch")]      = -0.05;
        // q_full[model.getDofIndex("r_ankle_pitch")]      = -0.05;
        // q_full[model.getDofIndex("l_shoulder_roll")]    = PI/8.0;
        // q_full[model.getDofIndex("r_shoulder_roll")]    = PI/8.0;


        taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 10.0, 2*sqrt(10.0), 0.0001, q_full);



    /* This part needs to be fixed... Not sure why but the orientation
        // Initialise waist pose

        // Eigen::Displacementd desiredWaistPose = Eigen::Displacementd(0.0,0.0,0.59,1.0,0.0,0.0,0.0);
        // taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "waistPoseTask", "root_link", ocra::XYZ, 36.0, 2*sqrt(36.0), 1.0, desiredWaistPose);
        */
        Eigen::Vector3d desiredWaistPosition, XYZdisp;
        desiredWaistPosition = model.getSegmentPosition(model.getSegmentIndex("root_link")).getTranslation();



        // XYZdisp << -0.03, 0.0, 0.0;
        // desiredWaistPosition = desiredWaistPosition + XYZdisp;

        // taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "waistPoseTask", "root_link", ocra::XYZ, 36.0, 2*sqrt(36.0), 1.0, desiredWaistPosition);


        // Initialise partial posture task


        // Eigen::VectorXi sdofs(3);
        // sdofs << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");
        // Eigen::VectorXd zero = Eigen::VectorXd::Zero(3);
        // taskManagers["tmPartialBack"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureBackTask", ocra::FullState::INTERNAL, sdofs, 16.0, 2*sqrt(16.0), 0.001, zero);

        double mu_sys = 0.5;
        double margin = 0.0;
        double sqrt2on2 = sqrt(2.0)/2.0;

        Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
        Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);


        // Initialise left foot contacts
        std::vector<Eigen::Displacementd> LFContacts;
        Eigen::Displacementd* PtrLFContacts = &LFContacts.front();
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown));
        taskManagers["tmFootContactLeft"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_sole", PtrLFContacts, LFContacts.size(), mu_sys, margin, false);

        // Initailise right foot contacts
        std::vector<Eigen::Displacementd> RFContacts;
        Eigen::Displacementd* PtrRFContacts = &RFContacts.front();
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown));
        taskManagers["tmFootContactRight"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_sole", PtrRFContacts, RFContacts.size(), mu_sys, margin, false);

    }

    void FloatingBaseMinimalTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {
    }

// }
