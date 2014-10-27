#ifndef ICUBCARTESIANTASKGENERATOR_H
#define ICUBCARTESIANTASKGENERATOR_H

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"
#include "orcisir/Tasks/ISIRTask.h"
#include "orcisir/Features/ISIRFeature.h"
#include <Eigen/Dense>
#include "orc/control/Model.h"

#include "ISIRCtrlTaskManager.h"


///===========================CoM Tasks===========================///
class iCubCoMTaskGenerator
{
public:

    iCubCoMTaskGenerator(ISIRCtrlTaskManager& tManager,
							const std::string& taskName, 
							Eigen::Vector3d target, 
							double stiffness, 
							double damping, 
							double weight);
    
    ~iCubCoMTaskGenerator();

    const std::string getTaskName();
    orc::PositionFeature* getFeature();
    orc::PositionFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::CoMFrame*              SF;
    orc::TargetFrame*           TF;
    orc::PositionFeature*       feat;
    orc::PositionFeature*       featDes;
    std::string                 segmentName;
    Eigen::Displacementd        posdes;
    Eigen::Twistd               veldes;
    const Model&                      model;
    ISIRCtrlTaskManager&            taskManager;

};



///===========================Cartesian Segment Task===========================///
class iCubCartesianTaskGenerator
{
public:


    iCubCartesianTaskGenerator(ISIRCtrlTaskManager& tManager,
								const std::string& taskName, 
								const std::string& segmentName,
								orc::ECartesianDof axes, 
								Eigen::Displacementd target, 
								double stiffness, 
								double damping, 
								double weight);
    
    ~iCubCartesianTaskGenerator();

    const std::string getTaskName();
    orc::PositionFeature* getFeature();
    orc::PositionFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::SegmentFrame*          SF;
    orc::TargetFrame*           TF;
    orc::PositionFeature*       feat;
    orc::PositionFeature*       featDes;
    std::string                 segmentName;
    Eigen::Displacementd        posdes;
    Eigen::Twistd               veldes;
    const Model&                      model;
    ISIRCtrlTaskManager&            taskManager;

};

class iCubOrientationTaskGenerator
{
public:


    iCubOrientationTaskGenerator(ISIRCtrlTaskManager& tManager,
                                const std::string& taskName,
                                const std::string& segmentName,
                                Eigen::Displacementd target,
                                double stiffness,
                                double damping,
                                double weight);

    ~iCubOrientationTaskGenerator();

    const std::string getTaskName();
    orc::OrientationFeature* getFeature();
    orc::OrientationFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::SegmentFrame*          SF;
    orc::TargetFrame*           TF;
    orc::OrientationFeature*       feat;
    orc::OrientationFeature*       featDes;
    std::string                 segmentName;
    Eigen::Displacementd        posdes;
    Eigen::Twistd               veldes;
    const Model&                      model;
    ISIRCtrlTaskManager&            taskManager;

};

///===========================Full/Partial Posture tasks===========================///
class iCubPostureTaskGenerator
{
public:


    iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
								const std::string& taskName,
								int state, 
								Eigen::VectorXd& target, 
								double stiffness, 
								double damping, 
								double weight);
    
    // Overloaded constructor for partial state posture tasks (torso stabilization for instance)
    iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
								const std::string& taskName,
								Eigen::VectorXi& selected_dof_indices,
								int state, 
								Eigen::VectorXd& target, 
								double stiffness, 
								double damping, 
								double weight);
    
    
    ~iCubPostureTaskGenerator();

    const std::string getTaskName();
    orc::FullStateFeature* getFeature();
    orc::FullStateFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::FullModelState*        FMS;
    orc::FullTargetState*       FTS;
    orc::FullStateFeature*      feat;
    orc::FullStateFeature*      featDes;
    
    orcisir::PartialModelState*   PMS;
    orcisir::PartialTargetState*  PTS;
    orcisir::PartialStateFeature* p_feat;
    orcisir::PartialStateFeature* p_featDes;
    
    Eigen::VectorXd             posdes;
    Eigen::Twistd               veldes;
    const Model&                     model;
    ISIRCtrlTaskManager&            taskManager;

};

class iCubContactTaskGenerator
{
public:

    iCubContactTaskGenerator(ISIRCtrlTaskManager& tManager,
                                   const std::string& taskName,
                                   const std::string& segmentName,
                                   Eigen::Displacementd H_segment_frame,
                                   double mu,
                                   double margin);

    ~iCubContactTaskGenerator();

    const std::string getTaskName();
    orc::PointContactFeature* getFeature();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask* task;
    orc::SegmentFrame* SF;
    orc::PointContactFeature* feat;
    std::string segmentName;
    const Model& model;
    ISIRCtrlTaskManager& taskManager;

};
#endif // ICUBCARTESIANTASKGENERATOR_H
