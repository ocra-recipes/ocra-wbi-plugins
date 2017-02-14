#ifndef _BOUNDING_H_
#define  _BOUNDING_H_

#include "walking-client/constraints/Constraint.h"

class Bounding : public Constraint {
public:
    Bounding();
    virtual ~Bounding();
protected:
    void buildMatrixCi();
    void buildMatrixCii();
    void buildVectord();
};

#endif
