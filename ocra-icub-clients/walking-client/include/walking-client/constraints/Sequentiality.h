#include "walking-client/constraints/Constraint.h"

class Sequentiality : public Constraint {
public:
  Sequentiality();
  virtual ~Sequentiality();
protected:
  void buildMatrixCi();
  void buildMatrixCii();
  void buildVectord();
};
