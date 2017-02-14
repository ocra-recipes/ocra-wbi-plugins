#ifndef _SINGLE_SUPPORT_H_
#define _SINGLE_SUPPORT_H_

#include "walking-client/constraints/constraint.h"

class SingleSupport : public Constraint {
private:
    Eigen::Vector2d _S;
public:
    SingleSupport();
    virtual ~SingleSupport();
protected:
    void buildMatrixCi();
    void buildMatrixCii();
    void buildVectord();
};

#endif
