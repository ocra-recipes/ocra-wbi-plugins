#ifndef ICUBTASKMANAGER_H
#define ICUBTASKMANAGER_H

// ORC INCLUDES
#include "orcisir/ISIRController.h"
#include "orc/control/Model.h"


// ORCISIR INCLUDES
#include "orcisir/Tasks/ISIRTask.h"
#include "orcisir/Constraints/ISIRConstraint.h"


class iCubTaskManager
{
public:

//===========================Constructor/Destructor===========================//
    iCubTaskManager(Model& model, orcisir::ISIRController& ctrl);
    virtual ~iCubTaskManager();

    void addTask2TaskList(orcisir::ISIRTask* task);

    const Model& getModel();
    const orcisir::ISIRController& getCtrl();

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};

#endif // ICUBTASKMANAGER_H
