#ifndef _CONTACTCONFIGENFORCEMENT_H_
#define _CONTACTCONFIGENFORCEMENT_H_

#include "walking-client/constraints/constraint.h"

class ContactConfigEnforcement : public Constraint {
private:

public:
    ContactConfigEnforcement ();
    virtual ~ContactConfigEnforcement ();
protected:
    void buildMatrixCi();
    void buildMatrixCii();
    void buildVectord();
};

#endif
