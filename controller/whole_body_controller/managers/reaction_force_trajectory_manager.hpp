#include <Eigen/Dense>

class ForceTask;
class PinocchioRobotSystem;

class ForceTrajectoryManager {
public:
  ForceTrajectoryManager(ForceTask *force_task, PinocchioRobotSystem *robot);
  ~ForceTrajectoryManager() = default;

  void InitializeInterpolation(const Eigen::VectorXd &des_init,
                               const Eigen::VectorXd &des_fin, double duration);
  void UpdateDesired(double query_time);

private:
  void _ConvertToLocalDesired(Eigen::VectorXd &des_rf);

  ForceTask *force_task_;
  PinocchioRobotSystem *robot_;

  Eigen::VectorXd des_init_rf_;
  Eigen::VectorXd des_final_rf_;
  double duration_;
};
