#include "walking-client/constraints/Constraint.h"

class Sequentiality : public Constraint {
public:
  Sequentiality();
  virtual ~Sequentiality();
protected:
  virtual void buildMatrixCi();
  virtual void buildMatrixCii();
  virtual void buildVectord();
};
