#ifndef _CONTACTCONFIGHISTORY_H_
#define _CONTACTCONFIGHISTORY_H_

#include "walking-client/constraints/constraint.h"

class ContactConfigHistory : public Constraint {
private:
public:
    ContactConfigHistory ();
    virtual ~ContactConfigHistory ();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};

#endif
