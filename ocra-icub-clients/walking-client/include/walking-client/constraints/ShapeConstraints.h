#ifndef _SHAPE_CONSTRAINTS_H_
#define  _SHAPE_CONSTRAINTS_H_

#include "walking-client/constraints/Constraint.h"
#include "walking-client/constraints/Bounding.h"
#include "walking-client/constraints/constancy.h"
#include "walking-client/constraints/sequentiality.h"

class ShapeConstraints : public Constraint {
protected:
  Bounding _bounding;
  Constancy _constancy;
  Sequentiality _sequentiality;
public:
  ShapeConstraints();
  virtual ~ShapeConstraints();
  virtual void buildMatrixCi();
  virtual void buildMatrixCii();
  virtual void buildVectord();
};
#endif
