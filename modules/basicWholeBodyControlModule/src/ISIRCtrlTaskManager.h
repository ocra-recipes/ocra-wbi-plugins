#ifndef ISIRCTRLTASKMANAGER_H
#define ISIRCTRLTASKMANAGER_H

// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"


// ORCISIR INCLUDES
#include "orcisir/Tasks/ISIRTask.h"
#include "orcisir/Constraints/ISIRConstraint.h"


class ISIRCtrlTaskManager
{
public:

//===========================Constructor/Destructor===========================//
    ISIRCtrlTaskManager();
    ISIRCtrlTaskManager(Model& model, orcisir::ISIRController& ctrl);
    virtual ~ISIRCtrlTaskManager();

    void addTask2TaskList(orcisir::ISIRTask* task);

    const Model& getModel();
    const orcisir::ISIRController& getCtrl();


private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};

#endif // ISIRCTRLTASKMANAGER_H
