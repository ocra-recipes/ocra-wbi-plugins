#ifndef _ADMISSIBILITY_CONSTRAINTS_H_
#define  _ADMISSIBILITY_CONSTRAINTS_H_

#include "walking-client/constraints/Constraint.h"
#include "walking-client/constraints/SSDSAlternation.h"
#include "walking-client/constraints/SingleSupport.h"
#include "walking-client/constraints/ContactConfigHistory.h"
#include "walking-client/constraints/ContactConfigEnforcement.h"

class AdmissibilityConstraints : public Constraint {
private:
protected:
    SingleSupport _singleSupport;
    SSDSAlternation _ssdsAlternation;
    ContactConfigHistory _contactConfigHistory;
    ContactConfigEnforcement _contactConfigEnforcement;
public:
    AdmissibilityConstraints();
    virtual ~AdmissibilityConstraints();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};
#endif
