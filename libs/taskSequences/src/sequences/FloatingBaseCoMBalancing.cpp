
#include <taskSequences/sequences/FloatingBaseCoMBalancing.h>

#include <ocraWbiPlugins/ocraWbiModel.h>

#include <math.h>



#define PI 3.14159265
#define TAU_MAX_LEG 12.0
#define TAU_MAX_ARM 8.0
#define TAU_MAX 24.0

// namespace sequence{
    void FloatingBaseCoMBalancing::doInit(wocra::wOcraController& c, wocra::wOcraModel& m)
    {
        ctrl = &c;
        model = &m;
        bool usesYARP = true;
        recorded = false;

        // Initialise full posture task
        Eigen::VectorXd q_full = model->getJointPositions();
        q_full[model->getDofIndex("l_knee")] = -PI / 12;
        q_full[model->getDofIndex("r_knee")] = -PI / 12;

        Eigen::VectorXd w_full = Eigen::VectorXd::Constant(model->nbInternalDofs(),0.01);
        w_full[model->getDofIndex("torso_pitch")] = 1;
        w_full[model->getDofIndex("torso_roll")] = 1;
        w_full[model->getDofIndex("torso_yaw")] = 1;

        taskManagers["fullPostureTask"] = new wocra::wOcraFullPostureTaskManager(*ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, 5.0, 2*sqrt(5.0), w_full, q_full, usesYARP);



        // Initialise com task
        Eigen::Vector3d initialCoMPosition;
        initialCoMPosition = model->getCoMPosition();
        taskManagers["CoMTask"] = new wocra::wOcraCoMTaskManager(*ctrl, *model, "CoMTask", ocra::XY, 50.0, 1*sqrt(50.0), 10.0, initialCoMPosition, usesYARP);
        tmCoM = dynamic_cast<wocra::wOcraCoMTaskManager*>(taskManagers["CoMTask"]);


//        taskManagers["CoMMomentumTask"]= new wocra::wOcraCoMMomentumTaskManager(*ctrl, *model, "CoMMomentumTask", ocra::XYZ, 2*sqrt(5.0), 10.0, usesYARP);
        // Initialise foot contacts
        double mu_sys = 1.0;
        double margin = 0.05;
        double sqrt2on2 = sqrt(2.0)/2.0;

        Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(sqrt2on2, 0.0, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
        Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, 0.0, -sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);


        std::vector<Eigen::Displacementd> LFContacts;
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotLZdown));
        LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotLZdown));
        taskManagers["leftFootContactTask"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "leftFootContactTask", "l_sole", LFContacts, mu_sys, margin);

        std::vector<Eigen::Displacementd> RFContacts;
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06,-0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-0.02, 0.02,0.0), rotRZdown));
        RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d( 0.06, 0.02,0.0), rotRZdown));
        taskManagers["rightFootContactTask"] = new wocra::wOcraContactSetTaskManager(*ctrl, *model, "rightFootContactTask", "r_sole", RFContacts, mu_sys, margin);

        // Activate Joint limit Constraint
        setJointLimits(0.1);

        // Activate Joint torque limit Constraint
        setJointTorqueLimits();

        applyWrench = true;

        portLFConctact.open("/FloatingBaseCoMBalancing/l_foot/getContactForce/rpc:o") ;    // Give it a name on the network.
        bool lfconnection;
        lfconnection = yarp::os::Network::connect("/FloatingBaseCoMBalancing/l_foot/getContactForce/rpc:o","/vectorView/l_foot/forceMeasurement/rpc:i");
        portRFConctact.open("/FloatingBaseCoMBalancing/r_foot/getContactForce/rpc:o");    // Give it a name on the network.
        bool rfconnection;
        rfconnection = yarp::os::Network::connect("/FloatingBaseCoMBalancing/r_foot/getContactForce/rpc:o","/vectorView/r_foot/forceMeasurement/rpc:i");
        if (!lfconnection)
            std::cout<<"[ERROR] lfoot rpc port connection" << std::endl;
        if (!rfconnection)
            std::cout<<"[ERROR] rfoot rpc port connection" << std::endl;

    }

    void FloatingBaseCoMBalancing::doUpdate(double time, wocra::wOcraModel& state, void** args)
    {

        double period = 6;
        double duration = 2*period;
        double ti = 0.2;
        if (time<ti)
            initialCoMPosition = model->getCoMPosition();
        else if (time>=ti && time<ti+duration)
        {
            double lf = model->getSegmentPosition(model->getSegmentIndex("l_foot")).getTranslation()[1];
            double rf = model->getSegmentPosition(model->getSegmentIndex("r_foot")).getTranslation()[1];


            double left = 0.9 * lf + 0.1 * rf;
            double right = 0.9 * rf + 0.1 * lf;
            double com_y, v_com_y, a_com_y;
            sinusoidalTraj(left, right, period, time-ti, com_y, v_com_y, a_com_y);


            Eigen::Vector3d desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration;
            Eigen::Vector3d actualCoMPosition = model->getCoMPosition();
            desiredCoMPosition << initialCoMPosition[0], com_y, initialCoMPosition[2];
            desiredCoMVelocity << 0, v_com_y, 0;
            desiredCoMAcceleration << 0, a_com_y, 0;

            tmCoM->setState(desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration);
            actual_com_x.push_back(actualCoMPosition[0]);
            ref_com_x.push_back(initialCoMPosition[0]);
            actual_com_y.push_back(actualCoMPosition[1]);
            ref_com_y.push_back(com_y);
            Eigen::Vector3d force;
            Eigen::Vector3d torque;
            force << 7, 7, 0;
            torque << 0, 0, 0;
            if (time >= ti+period && applyWrench)
            {
                applyExternalWrench("r_hand", force, torque, 1);
                applyWrench = false;
            }
            Eigen::Vector3d lfForce = getContactForce(&portLFConctact, "l_foot");
            Eigen::Vector3d rfForce = getContactForce(&portRFConctact, "r_foot");
            flf_x.push_back(lfForce[0]);
            flf_y.push_back(lfForce[1]);
            flf_z.push_back(lfForce[2]);

            frf_x.push_back(rfForce[0]);
            frf_y.push_back(rfForce[1]);
            frf_z.push_back(rfForce[2]);

            double kd = 2*sqrt(5.0);
            Eigen::Vector3d com_mom_error = model->getCoMAngularJacobian()*model->getAccelerationVariable().getValue() - kd * model->getCoMAngularVelocity();
            com_momentum_error_x.push_back(com_mom_error[0]);
            com_momentum_error_y.push_back(com_mom_error[1]);
            com_momentum_error_z.push_back(com_mom_error[2]);

        }

        else if (time >= ti+duration && !recorded)
        {
            saveCoMData();

            yarp::os::Network::disconnect("/FloatingBaseCoMBalancing/l_foot/getContactForce/rpc:o","/vectorView/l_foot/forceMeasurement/rpc:i");
            portLFConctact.close();
            yarp::os::Network::disconnect("/FloatingBaseCoMBalancing/r_foot/getContactForce/rpc:o","/vectorView/r_foot/forceMeasurement/rpc:i");
            portRFConctact.close();

            recorded = true;
        }
    }

    void FloatingBaseCoMBalancing::sinusoidalTraj(double left, double right, double period, double t, double& posTraj, double& velTraj, double& accTraj)
    {
        double c = 0.5 * (left + right);
        double a = 0.5 * fabs(right - left);
        double theta = 2 * PI * t/period;
        double dtheta = 2 * PI/period;
        posTraj = c + a * sin(theta);
        velTraj = a * cos(theta) * dtheta;
        accTraj = -a * dtheta * dtheta * sin(theta);
    }

    void FloatingBaseCoMBalancing::saveCoMData()
    {
        datafile.open ("./com_data.txt");

        for (std::vector<double>::iterator it = ref_com_y.begin() ; it != ref_com_y.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = actual_com_y.begin() ; it != actual_com_y.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = ref_com_x.begin() ; it != ref_com_x.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = actual_com_x.begin() ; it != actual_com_x.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = flf_x.begin() ; it != flf_x.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = flf_y.begin() ; it != flf_y.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = flf_z.begin() ; it != flf_z.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = frf_x.begin() ; it != frf_x.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = frf_y.begin() ; it != frf_y.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = frf_z.begin() ; it != frf_z.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = com_momentum_error_x.begin() ; it != com_momentum_error_x.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = com_momentum_error_y.begin() ; it != com_momentum_error_y.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        for (std::vector<double>::iterator it = com_momentum_error_z.begin() ; it != com_momentum_error_z.end(); ++it)
            datafile << *it <<" ";
        datafile<<"\n";

        datafile.close();
        std::cout<<"data saved."<<std::endl;
    }

    void FloatingBaseCoMBalancing::setJointTorqueLimits()
    {
        torqueSaturationLimit = Eigen::VectorXd::Constant(model->nbInternalDofs(),TAU_MAX);
        std::vector<std::string> legDofVector;
        legDofVector.push_back("l_hip_pitch");
        legDofVector.push_back("l_hip_roll");
        legDofVector.push_back("l_hip_yaw");
        legDofVector.push_back("l_knee");
        legDofVector.push_back("l_ankle_pitch");
        legDofVector.push_back("l_ankle_roll");
        legDofVector.push_back("r_hip_pitch");
        legDofVector.push_back("r_hip_roll");
        legDofVector.push_back("r_hip_yaw");
        legDofVector.push_back("r_knee");
        legDofVector.push_back("r_ankle_pitch");
        legDofVector.push_back("r_ankle_roll");
        for (std::vector<std::string>::iterator it = legDofVector.begin() ; it != legDofVector.end(); ++it)
            torqueSaturationLimit[model->getDofIndex(*it)] = TAU_MAX_LEG;
        std::vector<std::string> armDofVector;
        armDofVector.push_back("l_shoulder_pitch");
        armDofVector.push_back("l_shoulder_roll");
        armDofVector.push_back("l_shoulder_yaw");
        armDofVector.push_back("l_elbow");
        armDofVector.push_back("l_wrist_prosup");
        armDofVector.push_back("l_wrist_pitch");
        armDofVector.push_back("l_wrist_yaw");
        armDofVector.push_back("r_shoulder_pitch");
        armDofVector.push_back("r_shoulder_roll");
        armDofVector.push_back("r_shoulder_yaw");
        armDofVector.push_back("r_elbow");
        armDofVector.push_back("r_wrist_prosup");
        armDofVector.push_back("r_wrist_pitch");
        armDofVector.push_back("r_wrist_yaw");
        for (std::vector<std::string>::iterator it = armDofVector.begin() ; it != armDofVector.end(); ++it)
            torqueSaturationLimit[model->getDofIndex(*it)] = TAU_MAX_ARM;

        tauLimitConstraint = new wocra::TorqueLimitConstraint(*model,torqueSaturationLimit);
        ctrl->addConstraint(*tauLimitConstraint);
    }

    void FloatingBaseCoMBalancing::setJointLimits(double hpos)
    {
        jlConstraint = new wocra::JointLimitConstraint(*model, model->getJointLowerLimits(), model->getJointUpperLimits(), hpos);
        ctrl->addConstraint(*jlConstraint);
    }

    void FloatingBaseCoMBalancing::applyExternalWrench(std::string linkName, Eigen::Vector3d& force, Eigen::Vector3d& torque, double duration)//
    {

        yarp::os::RpcClient p;       // Create a port.
        p.open("/applyExternalWrench/rpc:o");    // Give it a name on the network.
        yarp::os::Network::connect("/applyExternalWrench/rpc:o","/icubGazeboSim/applyExternalWrench/rpc:i");

        yarp::os::Bottle cmd, ack;   // Make places to store things.
        cmd.addString (linkName);              // prepare command "cmd".
        cmd.addDouble ( force[0] );
        cmd.addDouble ( force[1] );
        cmd.addDouble ( force[2] );
        cmd.addDouble ( torque[0] );
        cmd.addDouble ( torque[1] );
        cmd.addDouble ( torque[2] );
        cmd.addDouble ( duration );
        p.write(cmd,ack); // send command, wait for reply.
        std::cout<<"force applied"<<std::endl;
        std::cout<<ack.get ( 0 ).asString()<<std::endl;// process response "ack".

        cmd.clear();
        ack.clear();
        yarp::os::Network::disconnect("/applyExternalWrench/rpc:o","/icubGazeboSim/applyExternalWrench/rpc:i");
        p.close();
    }

    Eigen::Vector3d& FloatingBaseCoMBalancing::getContactForce(yarp::os::RpcClient* p, std::string linkName)//
    {
//        Eigen::Vector3d force;

//        yarp::os::RpcClient p;       // Create a port.
//        p.open("/FloatingBaseCoMBalancing/"+linkName+"/getContactForce/rpc:o");    // Give it a name on the network.
//        yarp::os::Network::connect("/FloatingBaseCoMBalancing/"+linkName+"/getContactForce/rpc:o","/vectorView/"+linkName+"/forceMeasurement/rpc:i");

        yarp::os::Bottle cmd, ack;   // Make places to store things.
        cmd.addString (linkName);              // prepare command "cmd".
        p->write(cmd,ack); // send command, wait for reply.

        if ( ( ack.get ( 0 ).isDouble() || ack.get ( 0 ).isInt() ) \
             && ( ack.get ( 1 ).isDouble() || ack.get ( 1 ).isInt() ) \
             && ( ack.get ( 2 ).isDouble() || ack.get ( 2 ).isInt() ))
        {
            force[0]=ack.get ( 0 ).asDouble();
            force[1]=ack.get ( 1 ).asDouble();
            force[2]=ack.get ( 2 ).asDouble();
        }
        else
        {
            std::cout<<ack.get ( 0 ).asString()<<std::endl;// process response "ack".
        }
        cmd.clear();
        ack.clear();
//        yarp::os::Network::disconnect("/FloatingBaseCoMBalancing/getContactForce/rpc:o","/vectorView/forceMeasurement/rpc:i");
//        p.close();

        return force;
    }

// }
