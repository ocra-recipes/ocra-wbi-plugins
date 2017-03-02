/**
 *  \class MIQPState
 *  \brief Takes care of updating the state defined for the MIQP problem.
 *  \author Jorhabib Eljaik
 *  \cite ibanezThesis2015
 *  \warning Work in progress! This is still a barebone class in the process of being tested.
 *  \details
 **/

#ifndef _MIQP_STATE_H_
#define _MIQP_STATE_H_

#include <Eigen/Dense>
#include <ocra-icub/OcraWbiModel.h>
#include <ocra/util/ErrorsHelper.h>
#include "walking-client/utils.h"
#include <ocra-icub/Utilities.h>

namespace MIQP{
    enum StateVectorIndex{
        A_X=0,
        A_Y,
        B_X,
        B_Y,
        ALPHA_X,
        ALPHA_Y,
        BETA_X,
        BETA_Y,
        DELTA,
        GAMMA,
        H_X,
        H_Y,
        DH_X,
        DH_Y,
        DDH_X,
        DDH_Y
    };


    class MIQPState {
    private:
        /**
         * Full state \f$\mathbf{\xi}_k = (\mathbf{a} \; \mathbf{b} \; \mathbf{\alpha} \; \mathbf{\beta} \; \delta \; \gamma \; \mathbf{h} \; \mathbf{\dot{h}} \; \mathbf{\ddot{h}} )\f$
         */
        Eigen::VectorXd _xi_k;
        /**
         * Upper bounds \f$\mathbf{a} = (a_x, a_y)\f$. Its coordinates coincide with foot
         * center coordinates.
         */
        Eigen::Vector2d _a;
        /**
         * Lower bounds \f$\mathbf{b} = (b_x, b_y)\f$. It scoordinates coincide with foot
         * center coordinates.
         */
        Eigen::Vector2d _b;
        /**
         * Rising/Falling edge indicator for \f$\mathbf{a}\f$ (#_a)
         */
        Eigen::Vector2d _alpha;
        /**
         * Rising/Falling edge indicator for \f$\mathbf{b}\f$ (#_b)
         */
        Eigen::Vector2d _beta;
        /**
         * Binary variable indicating potential changes in \f$\mathbf{a}\f$ and \f$\mathbf{b}\f$
         * from DS to SS.
         *
         * If \f$\alpha_x\f$ and \f$\alpha_y\f$ change or \f$\beta_x\f$ and \f$\beta_y\f$ change,
         * then \f$\delta = 1\f$.
         *
         * If \f$\alpha_x\f$ and \f$\beta_y\f$ or \f$\beta_x\f$ and \f$\alpha_y\f$ change,
         * then \f$\delta = 0\f$.
         */
        unsigned int          _delta;

        /**
         * Single/Double Support indicator.
         * If the robot is in Double Support, then \f$\delta = 1\f$
         * If the robot is in Single Support, then \f$\delta = 0\f$
         */
        unsigned int          _gamma;

        /**
         * Horizonal CoM state, i.e. \f$\hat{\mathbf{h}}_k = [\mathbf{h}\;\dot{\mathbf{h}}\;\ddot{\mathbf{h}}]\f$.
         */
        Eigen::VectorXd _hk;

        /**
         * Robot model object necessary to update all the variables of the state vector.
         * Robot state will be queried through this object.
         */
        ocra::Model::Ptr _robotModel;

        /**
         * Robot prefix to be used (icubGazeboSim, icub)
         */
        std::string _robot;

        /*
         * Right foot coordinates
         */
        Eigen::Vector3d _r_foot_coord;

        /*
         * Left foot coordinates
         */
        Eigen::Vector3d _l_foot_coord;

        /*
         * Port that connects to the left foot F/T sensor.
         */
        yarp::os::BufferedPort<yarp::sig::Vector> _portWrenchLeftFoot;

        /*
         * Port that connect to the right F/T sensor.
         */
        yarp::os::BufferedPort<yarp::sig::Vector> _portWrenchRightFoot;

        /*
         * Normal force threshold
         */
        double _FzThreshold; // N

        /*
         * Normal distance threshold
         */
        double _PzThreshold; // m

    public:

        MIQPState (ocra::Model::Ptr robotModel);

        virtual ~MIQPState ();

        /**
         * Open and connects ports to F/T sensors in the feet of the robot and sets some thresholds.
         *
         * @return True if everything ends successfully.
         */
        bool initialize();

        /**
         * Calls the update of all the base of support descriptors and CoM state, and fills
         * #_xi_k with it. In order to retrieve, call MIQPState::getFullState().
         */
        void updateStateVector();

        /**
         Updates the base of support (BoS) descriptors, i.e. #_a, #_b, #_alpha, #_beta, #_delta, #_gamma.

         @param[out] a Upper bounds variable.
         @param[out] b Lower bounds variable.
         @param[out] alpha Rising/falling edges of #a.
         @param[out] beta Rising/falling edges of #b.
         @param[out] delta Potential change from DS to SS.
         @param[out] gamma SS/DS indicator.
         @param thresholdChange Threshold used to detect a change in #a and #b.
         */
        void updateBaseOfSupportDescriptors(Eigen::Vector2d &a,
                                            Eigen::Vector2d &b,
                                            Eigen::Vector2d &alpha,
                                            Eigen::Vector2d &beta,
                                            unsigned int &delta,
                                            unsigned int &gamma,
                                            double thresholdChange);

        /**
         * Updates the horizontal CoM state (horizonal position, velocity and acceleration).

         @param[out] hk 6-dimensional CoM state vector.
         */
        void updateHorizontalCoMState(Eigen::VectorXd &hk);

        /**
         * This method uses the normal measurements of the F/T sensors in the feet of the robot along with the \f$z\f$ coordinates of both feet to identify whether the robot is in DS or not.
         *
         * @param[out] If the robot is in SS, this variable contains the foot in SS.
         * @return True if robot is found in SS, false if it is in DS.
         */
        bool isRobotInSS(FOOT &footInSS);

        /**
         * Determins if #whichFoot is in contact with the ground by checking the normal F/T measurement of the corresponding foot
         * in combination with the \f$z\f$ coordinate.

         @param whichFoot Foot for which you want to know whether it's in contact or not.
         @param FzThreshold Z Force threshold to consider whether the foot is in contact or not.
         @param PzThreshold Z position threshold to consider whether the foot is in contact or not.
         @return True if #whichFoot is in contact. False otherwise.
         */
        bool isFootInContact(FOOT whichFoot, double FzThreshold, double PzThreshold);

        /**
         Reads the raw wrench published for the corresponding analog force/torque sensors in iCub's feet.

         @param whichFoot For which foot is the measurement read.
         @param rawWrench Result of the reading.
         @return True if reading is successful, false otherwise.
         */
        bool readFootWrench(FOOT whichFoot, Eigen::VectorXd &rawWrench);

        /**
         * Copies in #xi the full state #_xi_k.
         *
         * @param[out] xi Full state.
         */
        void getFullState(Eigen::VectorXd &xi);

        
        /**
         * Retrieves the translational part of the left foot position w.r.t world reference frame.
         *
         * @return Left foot 3D position
         */
        Eigen::Vector3d getLeftFootPosition();
        
        
        /**
         * Retrieves the translational part of the right foot position w.r.t. world reference frame.
         *
         * @return Right foot 3D position.
         */
        Eigen::Vector3d getRightFootPosition();
        
        /**
         * Operator to write the current state contents in a "pretty" way.
         */
        friend std::ostream& operator<<(std::ostream &out, const MIQPState &state) {
            out << "----- State Vector xi_k ----- \n";
            out << "\ta    : [" << state._a.transpose()               << "]\n";
            out << "\tb    : [" << state._b.transpose()               << "]\n";
            out << "\talpha: [" << state._alpha.transpose()           << "]\n";
            out << "\tbeta : [" << state._beta.transpose()            << "]\n";
            out << "\tdelta:  " << state._delta                       << "]\n";
            out << "\tgamma:  " << state._gamma                       << " \n";
            out << "\th    : [" << state._hk.head(2).transpose()       << " \n";
            out << "\tdh   : [" << state._hk.segment<2>(2).transpose()<< "]\n";
            out << "\tddh  : [" << state._hk.tail(2).transpose()      << "]\n";
            return out;
        }

    };
} // end of namespace
#endif
