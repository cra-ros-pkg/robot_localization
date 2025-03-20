#include "Eigen/Dense"
#include <robot_localization/filter_common.hpp>

namespace robot_localization {

class JacobianCreator {
public:
  void copyState(Eigen::VectorXd& state);

  virtual Eigen::MatrixXd createTransferFunction(double delta) = 0;

  virtual Eigen::MatrixXd createTransferJacobian(Eigen::MatrixXd& transfer_func) = 0;

protected:
  double delta_;

  double sp_;
  double cp_;
  double cpi_; 
  double tp_;

  double sr_;
  double cr_;

  double sy_;
  double cy_;

  double x_vel_;
  double y_vel_;
  double z_vel_;
  double pitch_vel_;
  double yaw_vel_;
  double x_acc_;
  double y_acc_;
  double z_acc_;
};

class OmniBasedJacobianCreator : public JacobianCreator {
public:
  Eigen::MatrixXd createTransferFunction(double delta) override;

  Eigen::MatrixXd createTransferJacobian(Eigen::MatrixXd& transfer_func) override;
};

class DiffBasedJacobianCreator : public JacobianCreator {
public:
  Eigen::MatrixXd createTransferFunction(double delta) override;

  Eigen::MatrixXd createTransferJacobian(Eigen::MatrixXd& transfer_func) override;
};

} // namespace robot_localization