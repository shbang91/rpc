#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class PinocchioRobotSystem;
class DracoStateProvider;

class DracoCAMTask : public Task {
public:
  DracoCAMTask(PinocchioRobotSystem *robot);
  ~DracoCAMTask() = default;

  void UpdateOpCommand() override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const bool b_sim) override;

private:
  DracoStateProvider *sp_;
};
