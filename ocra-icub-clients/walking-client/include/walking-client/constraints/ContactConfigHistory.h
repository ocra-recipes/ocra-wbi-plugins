#ifndef _CONTACTCONFIGHISTORY_H_
#define _CONTACTCONFIGHISTORY_H_

#include "walking-client/constraints/constraint.h"

class ContactConfigHistory : public Constraint {
private:
public:
    ContactConfigHistory ();
    virtual ~ContactConfigHistory ();
protected:
    void buildMatrixCi();
    void buildMatrixCii();
    void buildVectord();
};

#endif
