#pragma once

class Task;
class PinocchioRobotSystem;
class EndEffectorTrajectoryManager {
public:
  EndEffectorTrajectoryManager(Task *pos_task, Task *ori_task,
                               PinocchioRobotSystem *robot);
  virtual ~EndEffectorTrajectoryManager() = default;

  void UseCurrent();

private:
  Task *pos_task_;
  Task *ori_task_;
  PinocchioRobotSystem *robot_;
};
