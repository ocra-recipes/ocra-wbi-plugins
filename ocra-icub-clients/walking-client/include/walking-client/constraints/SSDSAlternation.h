#ifndef _SSDSALTERNATION_H
#define _SSDSALTERNATION_H

#include "walking-client/constraints/Constraint.h"

class SSDSAlternation : public Constraint {
public:
    SSDSAlternation();
    virtual ~SSDSAlternation();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};

#endif
