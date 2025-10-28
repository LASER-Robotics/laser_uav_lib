#include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>

namespace laser_uav_lib
{
// Usa os mesmos nomes de template do ficheiro .hpp (States, Inputs, Measurements)
template <int States, int Inputs, int Measurements>
EKF<States, Inputs, Measurements>::EKF() {
  x_.setZero();
  P_.setIdentity();
  F_.setIdentity();
  H_.setZero();
  Q_.setIdentity();
  R_.setIdentity();
  z_.setZero();
}

template <int States, int Inputs, int Measurements>
void EKF<States, Inputs, Measurements>::predict(const Eigen::Matrix<double, Inputs, 1> &u, double dt) {
  calculate_jacobian_F(x_, u, dt, F_);
  P_ = F_ * P_ * F_.transpose() + Q_;

  // CORRIGIDO: Usa 'States' em vez de 'S' para ser consistente com o template
  Eigen::Matrix<double, States, 1> next_x;
  state_transition_model(x_, u, dt, next_x);
  x_ = next_x;
}

template <int States, int Inputs, int Measurements>
void EKF<States, Inputs, Measurements>::correct(const Eigen::Matrix<double, Measurements, 1> &z) {
  z_ = z;  // Armazena a medição
  calculate_jacobian_H(x_, H_);

  Eigen::Matrix<double, Measurements, 1> z_pred;
  measurement_model(x_, z_pred);
  Eigen::Matrix<double, Measurements, 1> y = z_ - z_pred;

  // Variável 'S' renomeada para 'S_matrix' para evitar conflito
  Eigen::Matrix<double, Measurements, Measurements> S_matrix = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<double, States, Measurements>       K        = P_ * H_.transpose() * S_matrix.inverse();

  x_ += K * y;
  // Variável 'I' renomeada para 'I_matrix' para evitar conflito
  Eigen::Matrix<double, States, States> I_matrix = Eigen::Matrix<double, States, States>::Identity();
  P_                                             = (I_matrix - K * H_) * P_;
}

// Instanciação explícita para o DroneEKF
template class EKF<13, 4, 13>;

// Instanciação explícita para StateEstimator com tamanhos dinâmicos
template class EKF<13, Eigen::Dynamic, Eigen::Dynamic>;
}  // namespace laser_uav_lib