#include "robot_localization/jacobian_creators.hpp"

namespace robot_localization {

void JacobianCreator::copyState(Eigen::VectorXd& state)
{
  double roll = state(StateMemberRoll);
  double pitch = state(StateMemberPitch);
  double yaw = state(StateMemberYaw);

  // We'll need these trig calculations a lot.
  sp_ = std::sin(pitch);
  cp_ = std::cos(pitch);
  cpi_ = 1.0 / cp_;
  tp_ = sp_ * cpi_;

  sr_ = std::sin(roll);
  cr_ = std::cos(roll);

  sy_ = std::sin(yaw);
  cy_ = std::cos(yaw);

  x_vel_ = state(StateMemberVx);
  y_vel_ = state(StateMemberVy);
  z_vel_ = state(StateMemberVz);
  pitch_vel_ = state(StateMemberVpitch);
  yaw_vel_ = state(StateMemberVyaw);
  x_acc_ = state(StateMemberAx);
  y_acc_ = state(StateMemberAy);
  z_acc_ = state(StateMemberAz);
}

Eigen::MatrixXd OmniBasedJacobianCreator::createTransferFunction(double delta)
{
  delta_ = delta;

  Eigen::MatrixXd transfer_function;

  transfer_function(StateMemberX, StateMemberVx)  = cy_ * cp_ * delta_;
  transfer_function(StateMemberX, StateMemberVy)  = (cy_ * sp_ * sr_ - sy_ * cr_) * delta_;
  transfer_function(StateMemberX, StateMemberVz) = (cy_ * sp_ * cr_ + sy_ * sr_) * delta_;
  transfer_function(StateMemberX, StateMemberAx) = 0.5 * transfer_function(StateMemberX, StateMemberVx) * delta_;
  transfer_function(StateMemberX, StateMemberAy) = 0.5 * transfer_function(StateMemberX, StateMemberVy) * delta_;
  transfer_function(StateMemberX, StateMemberAz) = 0.5 * transfer_function(StateMemberX, StateMemberVz) * delta_;
  transfer_function(StateMemberY, StateMemberVx) = sy_ * cp_ * delta_;
  transfer_function(StateMemberY, StateMemberVy) = (sy_ * sp_ * sr_ + cy_ * cr_) * delta_;
  transfer_function(StateMemberY, StateMemberVz) = (sy_ * sp_ * cr_ - cy_ * sr_) * delta_;
  transfer_function(StateMemberY, StateMemberAx) = 0.5 * transfer_function(StateMemberY, StateMemberVx) * delta_;
  transfer_function(StateMemberY, StateMemberAy) = 0.5 * transfer_function(StateMemberY, StateMemberVy) * delta_;
  transfer_function(StateMemberY, StateMemberAz) = 0.5 * transfer_function(StateMemberY, StateMemberVz) * delta_;
  transfer_function(StateMemberZ, StateMemberVx) = -sp_ * delta_;
  transfer_function(StateMemberZ, StateMemberVy) = cp_ * sr_ * delta_;
  transfer_function(StateMemberZ, StateMemberVz) = cp_ * cr_ * delta_;
  transfer_function(StateMemberZ, StateMemberAx) = 0.5 * transfer_function(StateMemberZ, StateMemberVx) * delta_;
  transfer_function(StateMemberZ, StateMemberAy) = 0.5 * transfer_function(StateMemberZ, StateMemberVy) * delta_;
  transfer_function(StateMemberZ, StateMemberAz) = 0.5 * transfer_function(StateMemberZ, StateMemberVz) * delta_;
  transfer_function(StateMemberRoll, StateMemberVroll) = delta_;
  transfer_function(StateMemberRoll, StateMemberVpitch) = sr_ * tp_ * delta_;
  transfer_function(StateMemberRoll, StateMemberVyaw) = cr_ * tp_ * delta_;
  transfer_function(StateMemberPitch, StateMemberVpitch) = cr_ * delta_;
  transfer_function(StateMemberPitch, StateMemberVyaw) = -sr_ * delta_;
  transfer_function(StateMemberYaw, StateMemberVpitch) = sr_ * cpi_ * delta_;
  transfer_function(StateMemberYaw, StateMemberVyaw) = cr_ * cpi_ * delta_;
  transfer_function(StateMemberVx, StateMemberAx) = delta_;
  transfer_function(StateMemberVy, StateMemberAy) = delta_;
  transfer_function(StateMemberVz, StateMemberAz) = delta_;

  return transfer_function;
}

Eigen::MatrixXd OmniBasedJacobianCreator::createTransferJacobian(Eigen::MatrixXd& transfer_function)
{
  // Prepare the transfer function Jacobian. This function is analytically
  // derived from the transfer function.
  double x_coeff = 0.0;
  double y_coeff = 0.0;
  double z_coeff = 0.0;
  double one_half_at_squared = 0.5 * delta_ * delta_;

  y_coeff = cy_ * sp_ * cr_ + sy_ * sr_;
  z_coeff = -cy_ * sp_ * sr_ + sy_ * cr_;
  double dFx_dR = (y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (y_coeff * y_acc_ + z_coeff * z_acc_) * one_half_at_squared;
  double dFR_dR = 1.0 + (cr_ * tp_ * pitch_vel_ - sr_ * tp_ * yaw_vel_) * delta_;

  x_coeff = -cy_ * sp_;
  y_coeff = cy_ * cp_ * sr_;
  z_coeff = cy_ * cp_ * cr_;
  double dFx_dP =
    (x_coeff * x_vel_ + y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (x_coeff * x_acc_ + y_coeff * y_acc_ + z_coeff * z_acc_) *
    one_half_at_squared;
  double dFR_dP =
    (cpi_ * cpi_ * sr_ * pitch_vel_ + cpi_ * cpi_ * cr_ * yaw_vel_) * delta_;

  x_coeff = -sy_ * cp_;
  y_coeff = -sy_ * sp_ * sr_ - cy_ * cr_;
  z_coeff = -sy_ * sp_ * cr_ + cy_ * sr_;
  double dFx_dY =
    (x_coeff * x_vel_ + y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (x_coeff * x_acc_ + y_coeff * y_acc_ + z_coeff * z_acc_) *
    one_half_at_squared;

  y_coeff = sy_ * sp_ * cr_ - cy_ * sr_;
  z_coeff = -sy_ * sp_ * sr_ - cy_ * cr_;
  double dFy_dR = (y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (y_coeff * y_acc_ + z_coeff * z_acc_) * one_half_at_squared;
  double dFP_dR = (-sr_ * pitch_vel_ - cr_ * yaw_vel_) * delta_;

  x_coeff = -sy_ * sp_;
  y_coeff = sy_ * cp_ * sr_;
  z_coeff = sy_ * cp_ * cr_;
  double dFy_dP =
    (x_coeff * x_vel_ + y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (x_coeff * x_acc_ + y_coeff * y_acc_ + z_coeff * z_acc_) *
    one_half_at_squared;

  x_coeff = cy_ * cp_;
  y_coeff = cy_ * sp_ * sr_ - sy_ * cr_;
  z_coeff = cy_ * sp_ * cr_ + sy_ * sr_;
  double dFy_dY =
    (x_coeff * x_vel_ + y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (x_coeff * x_acc_ + y_coeff * y_acc_ + z_coeff * z_acc_) *
    one_half_at_squared;

  y_coeff = cp_ * cr_;
  z_coeff = -cp_ * sr_;
  double dFz_dR = (y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (y_coeff * y_acc_ + z_coeff * z_acc_) * one_half_at_squared;
  double dFY_dR = (cr_ * cpi_ * pitch_vel_ - sr_ * cpi_ * yaw_vel_) * delta_;

  x_coeff = -cp_;
  y_coeff = -sp_ * sr_;
  z_coeff = -sp_ * cr_;
  double dFz_dP =
    (x_coeff * x_vel_ + y_coeff * y_vel_ + z_coeff * z_vel_) * delta_ +
    (x_coeff * x_acc_ + y_coeff * y_acc_ + z_coeff * z_acc_) *
    one_half_at_squared;
  double dFY_dP =
    (sr_ * tp_ * cpi_ * pitch_vel_ + cr_ * tp_ * cpi_ * yaw_vel_) * delta_;


  // Much of the transfer function Jacobian is identical to the transfer
  // function
  Eigen::MatrixXd transfer_function_jacobian = transfer_function;
  transfer_function_jacobian(StateMemberX, StateMemberRoll) = dFx_dR;
  transfer_function_jacobian(StateMemberX, StateMemberPitch) = dFx_dP;
  transfer_function_jacobian(StateMemberX, StateMemberYaw) = dFx_dY;
  transfer_function_jacobian(StateMemberY, StateMemberRoll) = dFy_dR;
  transfer_function_jacobian(StateMemberY, StateMemberPitch) = dFy_dP;
  transfer_function_jacobian(StateMemberY, StateMemberYaw) = dFy_dY;
  transfer_function_jacobian(StateMemberZ, StateMemberRoll) = dFz_dR;
  transfer_function_jacobian(StateMemberZ, StateMemberPitch) = dFz_dP;
  transfer_function_jacobian(StateMemberRoll, StateMemberRoll) = dFR_dR;
  transfer_function_jacobian(StateMemberRoll, StateMemberPitch) = dFR_dP;
  transfer_function_jacobian(StateMemberPitch, StateMemberRoll) = dFP_dR;
  transfer_function_jacobian(StateMemberYaw, StateMemberRoll) = dFY_dR;
  transfer_function_jacobian(StateMemberYaw, StateMemberPitch) = dFY_dP;

  return transfer_function_jacobian;
}

Eigen::MatrixXd DiffBasedJacobianCreator::createTransferFunction(double delta)
{
  delta_ = delta;

  Eigen::MatrixXd transfer_function;

  transfer_function(StateMemberX, StateMemberVx) = cy_ * cp_ * delta_;
  transfer_function(StateMemberX, StateMemberAx) = 0.5 * transfer_function(StateMemberX, StateMemberVx) * delta_;
  transfer_function(StateMemberY, StateMemberVx) = sy_ * cp_ * delta_;
  transfer_function(StateMemberY, StateMemberAx) = 0.5 * transfer_function(StateMemberY, StateMemberVx) * delta_;
  transfer_function(StateMemberZ, StateMemberVx) = -sp_ * delta_;
  transfer_function(StateMemberZ, StateMemberAx) = 0.5 * transfer_function(StateMemberZ, StateMemberVx) * delta_;
  transfer_function(StateMemberRoll, StateMemberVroll) = delta_;
  transfer_function(StateMemberRoll, StateMemberVpitch) = sr_ * tp_ * delta_;
  transfer_function(StateMemberRoll, StateMemberVyaw) = cr_ * tp_ * delta_;
  transfer_function(StateMemberPitch, StateMemberVpitch) = cr_ * delta_;
  transfer_function(StateMemberPitch, StateMemberVyaw) = -sr_ * delta_;
  transfer_function(StateMemberYaw, StateMemberVpitch) = sr_ * cpi_ * delta_;
  transfer_function(StateMemberYaw, StateMemberVyaw) = cr_ * cpi_ * delta_;
  transfer_function(StateMemberVx, StateMemberAx) = delta_;
  transfer_function(StateMemberVy, StateMemberAy) = delta_;
  transfer_function(StateMemberVz, StateMemberAz) = delta_;

  return transfer_function;
}

Eigen::MatrixXd DiffBasedJacobianCreator::createTransferJacobian(Eigen::MatrixXd& transfer_function)
{
  // Prepare the transfer function Jacobian. This function is analytically
  // derived from the transfer function.
  double one_half_at_squared = 0.5 * delta_ * delta_;

  double dFR_dR = 1.0 + (cr_ * tp_ * pitch_vel_ - sr_ * tp_ * yaw_vel_) * delta_;

  double x_coeff = -cy_ * sp_;
  double dFx_dP =
    (x_coeff * x_vel_) * delta_ + (x_coeff * x_acc_) * one_half_at_squared;
  double dFR_dP = (cpi_ * cpi_ * sr_ * pitch_vel_ + cpi_ * cpi_ * cr_ * yaw_vel_) * delta_;

  x_coeff = -sy_ * cp_;
  double dFx_dY =
    (x_coeff * x_vel_) * delta_ + (x_coeff * x_acc_) * one_half_at_squared;

  double dFP_dR = (-sr_ * pitch_vel_ - cr_ * yaw_vel_) * delta_;

  x_coeff = -sy_ * sp_;
  double dFy_dP =
    (x_coeff * x_vel_) * delta_ + (x_coeff * x_acc_) * one_half_at_squared;

  x_coeff = cy_ * cp_;
  double dFy_dY =
    (x_coeff * x_vel_) * delta_ + (x_coeff * x_acc_) * one_half_at_squared;

  double dFY_dR = (cr_ * cpi_ * pitch_vel_ - sr_ * cpi_ * yaw_vel_) * delta_;

  x_coeff = -cp_;
  double dFz_dP =
    (x_coeff * x_vel_) * delta_ + (x_coeff * x_acc_) * one_half_at_squared;
  double dFY_dP =
    (sr_ * tp_ * cpi_ * pitch_vel_ + cr_ * tp_ * cpi_ * yaw_vel_) * delta_;


  // Much of the transfer function Jacobian is identical to the transfer
  // function
  Eigen::MatrixXd transfer_function_jacobian = transfer_function;
  transfer_function_jacobian(StateMemberX, StateMemberPitch) = dFx_dP;
  transfer_function_jacobian(StateMemberX, StateMemberYaw) = dFx_dY;
  transfer_function_jacobian(StateMemberY, StateMemberPitch) = dFy_dP;
  transfer_function_jacobian(StateMemberY, StateMemberYaw) = dFy_dY;
  transfer_function_jacobian(StateMemberZ, StateMemberPitch) = dFz_dP;
  transfer_function_jacobian(StateMemberRoll, StateMemberRoll) = dFR_dR;
  transfer_function_jacobian(StateMemberRoll, StateMemberPitch) = dFR_dP;
  transfer_function_jacobian(StateMemberPitch, StateMemberRoll) = dFP_dR;
  transfer_function_jacobian(StateMemberYaw, StateMemberRoll) = dFY_dR;
  transfer_function_jacobian(StateMemberYaw, StateMemberPitch) = dFY_dP;

  return transfer_function_jacobian;
}

} // namespace robot_localization