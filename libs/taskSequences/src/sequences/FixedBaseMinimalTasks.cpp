#include <taskSequences/sequences/FixedBaseMinimalTasks.h>
// FixedBaseMinimalTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
  void FixedBaseMinimalTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
  {
    for (unsigned int i = 0; i < NB_POSTURES; ++i)                  // NB_POSTURES elements
      q.push_back(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    raiseArm ('l', model, q.at(0));
    raiseArm ('r', model, q.at(1));
    extendArm('l', model, q.at(1));
    downArm  ('l', model, q.at(2));
    extendArm('r', model, q.at(2));
    raiseArm ('r', model, q.at(3));
    raiseArm ('l', model, q.at(3));
    downArm  ('r', model, q.at(4));
    downArm  ('r', model, q.at(4));

    p   = 20.0;
    t_d = 13.0;
    w   = 1.0;

    posture = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, p, t_d, w, q.at(0));
    taskManagers["tmFull"] = posture;
    mode = 0;
    count = 0;
  }

  // set functions
  // void setStiffness(double stiffness); = kp
  // void setDamping(double damping);     = td
  // void setWeight(double weight);
  void FixedBaseMinimalTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
  {
    // parameters iteration
    double time_step  = 20.0;
    int    max        = 10;
    double value_step = 5.0;
    double t          = time - max*time_step*floor(time/(max*time_step));
    if(t > time_step*(count+1) || (t < time_step && count == max - 1 ))
    {
      count     = (count + 1)%max;
      p         =     value_step*count;
      t_d       = 0.2*value_step*count;
      posture->setStiffness(p);
      posture->setDamping(t_d);
      posture->setWeight(w);

      std::cout << "Stiffness = "     << p
                << "; Damping = "     << t_d
                << "; at "            << time
                <<"s; Task weight = " << w
                << ";"                << std::endl;
    }

    // posture iteration, assures the sequence of NB_POSTURES postures at the vector q in a period of PERIOD seconds
    time -= PERIOD*floor(time/PERIOD); // normalized time

    if(time >= (mode + 1)*PERIOD/NB_POSTURES + 0.1  || (time < PERIOD/NB_POSTURES && mode == NB_POSTURES - 1))
    {
      posture->setPosture(q.at(mode));
      mode = (mode + 1)%NB_POSTURES;
    }
  }

// }
