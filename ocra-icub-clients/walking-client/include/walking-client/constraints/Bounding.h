#ifndef _BOUNDING_H_
#define  _BOUNDING_H_

#include "walking-client/constraints/Constraint.h"

class Bounding : public Constraint {
public:
    Bounding();
    virtual ~Bounding();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};

#endif
