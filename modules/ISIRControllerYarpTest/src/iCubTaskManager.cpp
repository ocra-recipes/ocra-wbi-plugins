#include "iCubTaskManager.h"
#include <vector>

struct iCubTaskManager::Pimpl
{
  const Model& model;
  orcisir::ISIRController& ctrl;
  Pimpl(Model& m, orcisir::ISIRController& c)
      :model(m)
      ,ctrl(c)
  {
  }

  ~Pimpl()
  {}
};

iCubTaskManager::iCubTaskManager(Model& model, orcisir::ISIRController& ctrl)
    :pimpl( new Pimpl(model, ctrl) )
{

}

iCubTaskManager::~iCubTaskManager()
{

}

void iCubTaskManager::addTask2TaskList(orcisir::ISIRTask* task)
{
    pimpl->ctrl.addTask(*task);

}


const Model& iCubTaskManager::getModel()
{
    return pimpl->model;
}

const orcisir::ISIRController&  iCubTaskManager::getCtrl()
{
    return pimpl->ctrl;
}
