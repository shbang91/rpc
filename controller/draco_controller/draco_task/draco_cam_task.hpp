#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class PinocchioRobotSystem;
class DracoStateProvider;

class DracoCAMTask : public Task {
public:
  DracoCAMTask(PinocchioRobotSystem *robot);
  ~DracoCAMTask() = default;

  void UpdateOpCommand(const Eigen::Matrix3d &world_R_local =
                           Eigen::Matrix3d::Identity()) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const WBC_TYPE wbc_type) override;

private:
  DracoStateProvider *sp_;
};
