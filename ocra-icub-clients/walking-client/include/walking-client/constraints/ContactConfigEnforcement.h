#ifndef _CONTACTCONFIGENFORCEMENT_H_
#define _CONTACTCONFIGENFORCEMENT_H_

#include "walking-client/constraints/constraint.h"

class ContactConfigEnforcement : public Constraint {
private:

public:
    ContactConfigEnforcement ();
    virtual ~ContactConfigEnforcement ();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};

#endif
