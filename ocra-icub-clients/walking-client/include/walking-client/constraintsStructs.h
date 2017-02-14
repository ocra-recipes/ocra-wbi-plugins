#ifndef _CONSTRAINTS_STRUCTS_H_
#define  _CONSTRAINTS_STRUCTS_H_

#include <Eigen/Core>

struct ShapeConstraintsStruct {
    struct boundingStruct {
        Eigen::MatrixXd C;
        Eigen::VectorXd d;
        Eigen::MatrixXd Cbar;
        Eigen::VectorXd dbar;
        Eigen::VectorXd fhistory;
    };
    struct constancyStruct {
        // Upper bound of the discontinuities in a and b. S = (Sx, Sy)
        Eigen::Vector2d S;
        Eigen::MatrixXd C1;
        Eigen::MatrixXd C2;
        Eigen::VectorXd d;
        Eigen::VectorXd dBar;
        Eigen::MatrixXd Cbar1;
        Eigen::MatrixXd Cbar2;
        Eigen::VectorXd fHistory;
    };
    struct sequentialityStruct {
        Eigen::MatrixXd C;
        Eigen::VectorXd d;
        Eigen::MatrixXd Cbar;
        Eigen::VectorXd dBar;
        Eigen::VectorXd fHistory;
    };

    Eigen::MatrixXd Cbar;
    Eigen::VectorXd dbar;
    Eigen::VectorXd fHistory;
    boundingStruct bounding;
    constancyStruct constancy;
    sequentialityStruct sequentiality;
};

struct AdmissibilityConstraintsStruct {
    struct SSandDSAlternation {
        Eigen::MatrixXd C1;
        Eigen::MatrixXd C2;
        Eigen::VectorXd d;
        Eigen::MatrixXd C1bar;
        Eigen::MatrixXd C2bar;
        Eigen::VectorXd dBar;
        Eigen::VectorXd fHistory;
    };
    struct SingleSupport {
        Eigen::MatrixXd C;
        Eigen::VectorXd d;
        Eigen::MatrixXd Cbar;
        Eigen::VectorXd dBar;
        Eigen::VectorXd S;
        Eigen::VectorXd fHistory;
    };
    struct ContactConfigurationHistory {
        Eigen::MatrixXd C1;
        Eigen::MatrixXd C2;
        Eigen::VectorXd d;
        Eigen::MatrixXd C1bar;
        Eigen::MatrixXd C2bar;
        Eigen::VectorXd dBar;
        Eigen::VectorXd fHistory;
    };
    struct ContactConfigurationEnforcement {
        Eigen::MatrixXd C;
        Eigen::VectorXd d;
        Eigen::MatrixXd Cbar;
        Eigen::VectorXd dBar;
        Eigen::VectorXd fHistory;
    };
    Eigen::MatrixXd Cbar;
    Eigen::VectorXd dbar;
    Eigen::VectorXd fHistory;
    SSandDSAlternation ssAndDoubleSupportAlternation;
    SingleSupport singleSupport;
    ContactConfigurationHistory contactConfigurationHistory;
    ContactConfigurationEnforcement contactConfigurationEnforcement;
};

enum CONSTRAINTS_TYPE {
    SHAPE_CONSTRAINTS = 0,
    ADMISSIBILITY_CONSTRAINTS,
    WALKING_CONSTRAINTS
};


#endif
