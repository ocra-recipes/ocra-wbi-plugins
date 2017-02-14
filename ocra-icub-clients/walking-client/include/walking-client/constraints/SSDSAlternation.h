#ifndef _SSDSALTERNATION_H
#define _SSDSALTERNATION_H

#include "walking-client/constraints/constraint.h"

class SSDSAlternation : public Constraint {
public:
    SSDSAlternation();
    virtual ~SSDSAlternation();
protected:
    void buildMatrixCi();
    void buildMatrixCii();
    void buildVectord();
};

#endif
