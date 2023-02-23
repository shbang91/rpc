#include <Eigen/Dense>

class ForceTask;
class PinocchioRobotSystem;

class ForceTrajectoryManager {
public:
  ForceTrajectoryManager(ForceTask *force_task, PinocchioRobotSystem *robot);
  ~ForceTrajectoryManager() = default;

  void InitializeInterpolation(const Eigen::VectorXd &des_init,
                               const Eigen::VectorXd &des_fin, double duration);
  void InitializeSwaying(const Eigen::VectorXd &init_des_force,
                         const Eigen::Vector3d &amp,
                         const Eigen::Vector3d &freq);
  void UpdateDesired(double query_time);

  Eigen::VectorXd GetFinalDesiredRf();

private:
  void _ConvertToLocalDesired(Eigen::VectorXd &des_rf);

  ForceTask *force_task_;
  PinocchioRobotSystem *robot_;

  Eigen::VectorXd des_init_rf_;
  Eigen::VectorXd des_final_rf_;
  double duration_;

  // com swaying
  Eigen::VectorXd grav_vec_;
  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;
  bool b_swaying_;
};
