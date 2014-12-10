#include "ISIRCtrlTaskManager.h"
#include <vector>

struct ISIRCtrlTaskManager::Pimpl
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

ISIRCtrlTaskManager::ISIRCtrlTaskManager()
{

}


ISIRCtrlTaskManager::ISIRCtrlTaskManager(Model& model, orcisir::ISIRController& ctrl)
    :pimpl( new Pimpl(model, ctrl) )
{

}

ISIRCtrlTaskManager::~ISIRCtrlTaskManager()
{

}

void ISIRCtrlTaskManager::addTask2TaskList(orcisir::ISIRTask* task)
{
    pimpl->ctrl.addTask(*task);

}


const Model& ISIRCtrlTaskManager::getModel()
{
    return pimpl->model;
}

const orcisir::ISIRController&  ISIRCtrlTaskManager::getCtrl()
{
    return pimpl->ctrl;
}
