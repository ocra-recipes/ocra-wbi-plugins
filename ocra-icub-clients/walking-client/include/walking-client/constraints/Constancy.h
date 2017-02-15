#include "walking-client/constraints/Constraint.h"

class Constancy : public Constraint {
private:
    /* Upper bounds */
    Eigen::Vector2d _S;
public:
    Constancy();
    virtual ~Constancy();
protected:
    virtual void buildMatrixCi();
    virtual void buildMatrixCii();
    virtual void buildVectord();
};
