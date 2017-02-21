#include "walking-client/MIQPState.h"


MIQPState::MIQPState(ocra::Model::Ptr robotModel):
_xi_k(Eigen::VectorXd(SIZE_STATE_VECTOR)),
_hk(Eigen::VectorXd(6)),
_robotModel(robotModel),
_robot("icubGazeboSim")
{
    initialize();
}

MIQPState::~MIQPState(){}

bool MIQPState::initialize() {
    
    // Connect to feet wrench ports
    bool ok = _portWrenchLeftFoot.open("/walkingClient/MIQPState/left_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/MIQPState/left_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        std::string src = std::string("/"+_robot+"/left_foot/analog:o");
        if (!yarp::os::Network::connect(src, _portWrenchLeftFoot.getName().c_str())) {
            OCRA_ERROR("Impossible to connect to " << src);
            return false;
        }
    }
    ok = _portWrenchRightFoot.open("/walkingClient/MIQPState/right_foot/wrench:i");
    if (!ok) {
        OCRA_ERROR("Impossible to open /walkingClient/MIQPState/right_foot/wrench:i");
        return false;
    } else {
        // Autoconnect
        std::string src = std::string("/"+_robot+"/right_foot/analog:o");
        if (!yarp::os::Network::connect(src, _portWrenchRightFoot.getName().c_str()) ) {
            OCRA_ERROR("Impossible to connect to " << src);
            return false;
        }
    }
    
    // Set thresholds
    // FIXME: This should come from configuration file
    _FzThreshold = 5; // N
    _PzThreshold = 0.05; // m => 5cm
    
    return true;

}

void MIQPState::updateStateVector() {
    /* TODO: This threshold should not be hardcoded but from config file*/
    double thresholdChange = 0.015; //1.5cm
    updateBaseOfSupportDescriptors(_a, _b, _alpha, _beta, _delta, _gamma, thresholdChange);
    updateHorizontalCoMState(_hk);
    _xi_k << _a, _b, _alpha, _beta, _delta, _gamma, _hk;
}

bool MIQPState::isRobotInSS(FOOT &footInSS) {
    // is left foot in contact?
    bool lFootInContact = isFootInContact(LEFT_FOOT, _FzThreshold, _PzThreshold);
    // is right foot in contact?
    bool rFootInContact = isFootInContact(RIGHT_FOOT, _FzThreshold, _PzThreshold);
    // if both are in contact, then return false
    if (lFootInContact && rFootInContact)
        return false;
    // if only one of the two is in contact, return in footInSS the foot in contact
    if (lFootInContact || rFootInContact) {
        if (lFootInContact)
            footInSS = LEFT_FOOT;
        if (rFootInContact)
            footInSS = RIGHT_FOOT;
        return true;
    }
    
    OCRA_ERROR("Something smells fishy! No feet found in contact!!!")
    return false;
        
}

bool MIQPState::isFootInContact(FOOT whichFoot, double FzThreshold, double PzThreshold) {
    // Read normal F/T sensor measurement and compare to a threshold. Do the same for the z coordinate of the corresponding foot.
    Eigen::VectorXd rawWrench(6);
    double coordZ;
    switch (whichFoot) {
        case LEFT_FOOT:
            readFootWrench(LEFT_FOOT, rawWrench);
            coordZ = _l_foot_coord(2);
            break;
        case RIGHT_FOOT:
            readFootWrench(RIGHT_FOOT, rawWrench);
            coordZ = _r_foot_coord(2);
            break;
        default:
            break;
    }
    
    if (rawWrench(2) < -_FzThreshold && coordZ < _PzThreshold)
        return true;
    else
        return false;
}

void MIQPState::updateBaseOfSupportDescriptors(Eigen::Vector2d &aa,
                                                Eigen::Vector2d &bb,
                                                Eigen::Vector2d &aalpha,
                                                Eigen::Vector2d &bbeta,
                                                unsigned int &ddelta,
                                                unsigned int &ggamma,
                                                double thresholdChange){
    // This method will actually update a, b, alpha, beta, delta and gamma
    // Retrieve coordinates of left sole
    _l_foot_coord = _robotModel->getSegmentPosition(_robotModel->getSegmentIndex("l_sole")).getTranslation();
    // Retrieve coordinates of right sole
    _r_foot_coord = _robotModel->getSegmentPosition(_robotModel->getSegmentIndex("r_sole")).getTranslation();
    // If the robot is in SS, identify which foot is on the ground and set lower and upper bounds accordingly
    Eigen::Vector2d a;
    Eigen::Vector2d b;
    FOOT footInSS;
    if (isRobotInSS(footInSS)) {
        switch (footInSS) {
            case LEFT_FOOT:
                a(0) = b(0) = _l_foot_coord(0);
                a(1) = b(1) = _l_foot_coord(1);
                break;
            case RIGHT_FOOT:
                a(0) = b(0) = _r_foot_coord(0);
                a(1) = b(1) = _r_foot_coord(1);
                break;
            default:
                break;
        }
        // UPDATE the state variable GAMMA
        ggamma = 0;
    } else {
        // Compare coordinates of both feet to find ax, ay, bx, by
        if (_l_foot_coord(0) > _r_foot_coord(0)) {
            a(0) = _l_foot_coord(0);
            b(0) = _r_foot_coord(0);
        } else {
            a(0) = _r_foot_coord(0);
            b(0) = _l_foot_coord(0);
        }
        if (_l_foot_coord(1) > _r_foot_coord(1)) {
            a(1) = _l_foot_coord(1);
            b(1) = _r_foot_coord(1);
        } else {
            a(1) = _r_foot_coord(1);
            b(1) = _l_foot_coord(1);
        }
        // UPDATE THE state variable GAMMA with DS
        ggamma = 1;
    }
    
    // UPDATING also ALPHA and BETA
    Eigen::Vector2d alpha;
    Eigen::Vector2d beta;
    std::abs(a(0) - aa(0)) > thresholdChange ? alpha(0) = 1 : alpha(0) = 0;
    std::abs(b(0) - bb(0)) > thresholdChange ? beta(0)  = 1 : beta(0)  = 0;
    std::abs(a(1) - aa(1)) > thresholdChange ? alpha(1) = 1 : alpha(1) = 0;
    std::abs(b(1) - bb(1)) > thresholdChange ? beta(1)  = 1 : beta(1)  = 0;
    
    // UPDATE state variables a and b
    aa = a;
    bb = b;

    // UPDATE delta
    /* TODO: Check if the update of delta is well done! */
    if ((alpha(0) == 1 && alpha(1) == 1) || (beta(0) == 1 && beta(1) == 1))
        ddelta = 1;
    if ((alpha(0) == 1 && beta(1) == 1) || (beta(0) == 1 && alpha(1) == 1))
        ddelta = 0;
    
    // ACTUALLY UPDATE aalpa, bbeta
    aalpha = alpha;
    bbeta = beta;
}

void MIQPState::updateHorizontalCoMState(Eigen::VectorXd &hk) {
    hk.head<2>() = _robotModel->getCoMPosition().topRows(2);
    hk.segment<2>(2) = _robotModel->getCoMVelocity().topRows(2);
    hk.tail<2>() = _robotModel->getCoMAcceleration().topRows(2);
}

bool MIQPState::readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench) {
    yarp::sig::Vector * yRawFootWrench;
    switch (whichFoot) {
        case LEFT_FOOT:
            yRawFootWrench = _portWrenchLeftFoot.read();
            break;
        case RIGHT_FOOT:
            yRawFootWrench = _portWrenchRightFoot.read();
            break;
        default:
            break;
    }
    
    if (yRawFootWrench == NULL)
        return false;
    
    rawWrench = Eigen::VectorXd::Map(yRawFootWrench->data(), 6);
    return true;
}

void MIQPState::getFullState(Eigen::VectorXd &xi) {
    xi = _xi_k;
}

std::ostream& operator<<(std::ostream &out, const MIQPState &state) {
    out << "----- State Vector xi_k ----- \n";
    out << "a: " << state._a.transpose() << "\n";
    out << "b: " << state._b.transpose() << "\n";
    out << "alpha: " << state._alpha.transpose() << "\n";
    out << "beta: " << state._beta.transpose() << "\n";
    out << "delta: " << state._delta << "\n";
    out << "gamma: " << state._gamma << "\n";
    out << "h: " << state._hk.head(2) << "\n";
    out << "dh: " << state._hk.segment<2>(2) << "\n";
    out << "ddh: " << state._hk.tail(2) << "\n";
    return out;
}


