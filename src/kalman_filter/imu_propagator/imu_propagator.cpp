#include "laser_uav_lib/kalman_filter/imu_propagator/imu_propagator.hpp"
#include <iostream>

namespace laser_uav_lib
{

ImuPropagator::ImuPropagator() {
  state_.setZero();
  state_(StateIMU::QW) = 1.0;  // Inicializa Quaternião como identidade

  cov_.setIdentity();
  cov_ *= 1e-4;

  // Valores padrão (serão sobrescritos pelo set_noise_coefficients se chamado)
  noise_gyr_      = 0.00018665;
  noise_acc_      = 0.00186;
  noise_bias_gyr_ = 0.0087;
  noise_bias_acc_ = 0.1960;

  // Gravidade
  // NOTA: Se o referencial for ENU, a gravidade física é [0, 0, -9.81].
  // Aqui estamos a definir o vetor a SUBTRAIR da medição do acelerômetro.
  // Se o acc mede +9.81 quando parado, subtrair +9.81 resulta em 0 aceleração.
  gravity_ << 0, 0, 9.81;
}

ImuPropagator::~ImuPropagator() {
}

// --- MÉTODOS DE PROPAGAÇÃO ---

void ImuPropagator::propagate(const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &linear_acceleration, double dt) {

  // --- 1. Recuperar Estado Atual ---
  Eigen::Quaterniond q_IG(state_(StateIMU::QW), state_(StateIMU::QX), state_(StateIMU::QY), state_(StateIMU::QZ));
  q_IG.normalize();  // Garantir normalização

  Eigen::Vector3d bg = state_.segment<3>(StateIMU::BGX);
  Eigen::Vector3d v  = state_.segment<3>(StateIMU::VX);
  Eigen::Vector3d ba = state_.segment<3>(StateIMU::BAX);
  Eigen::Vector3d p  = state_.segment<3>(StateIMU::PX);

  // --- 2. Remover Bias das Medições ---
  Eigen::Vector3d w_hat = angular_velocity - bg;
  Eigen::Vector3d a_hat = linear_acceleration - ba;

  // Matrizes de Rotação (Body -> Global)
  Eigen::Matrix3d C_IG = q_IG.toRotationMatrix();
  Eigen::Matrix3d C_GI = C_IG.transpose();  // Nota: O artigo define q_IG como Global->IMU ou IMU->Global?
                                            // Geralmente q_WB (Body to World).
                                            // Assumindo aqui que q_IG representa rotação do corpo.
                                            // Se q_IG é quaternion de atitude normal, R = q.matrix() converte Body->World.
                                            // Cuidado: No teu código original tinhas C_GI = C_IG.transpose().
                                            // Se acc_global = C_GI * a_hat, então C_GI deve ser R_Body_to_World.
                                            // O padrão Eigen q.toRotationMatrix() dá R_Body_to_World.
                                            // Vou assumir que queres R_Body_to_World para projetar a aceleração.

  Eigen::Matrix3d R_body_to_world = q_IG.toRotationMatrix();

  // --- 3. Propagação do Estado Nominal (Integração de Euler) ---

  // Aceleração Global (removendo gravidade)
  Eigen::Vector3d acc_global = R_body_to_world * a_hat - gravity_;

  // Posição: p_k+1 = p_k + v_k*dt + 0.5*a*dt^2
  p = p + v * dt + 0.5 * acc_global * dt * dt;

  // Velocidade: v_k+1 = v_k + a*dt
  v = v + acc_global * dt;

  // Rotação: q_k+1 = q_k * exp(0.5 * w * dt)
  // Aproximação para pequenos ângulos
  Eigen::Vector3d    delta_theta = w_hat * dt;
  Eigen::Quaterniond dq;
  dq.w()   = 1.0;
  dq.vec() = 0.5 * delta_theta;
  q_IG     = (q_IG * dq.normalized()).normalized();

  // Atualizar vetor de estado
  state_(StateIMU::QW)             = q_IG.w();
  state_(StateIMU::QX)             = q_IG.x();
  state_(StateIMU::QY)             = q_IG.y();
  state_(StateIMU::QZ)             = q_IG.z();
  state_.segment<3>(StateIMU::BGX) = bg;
  state_.segment<3>(StateIMU::VX)  = v;
  state_.segment<3>(StateIMU::BAX) = ba;
  state_.segment<3>(StateIMU::PX)  = p;

  // --- 4. Matriz Jacobiana F_c (Estado de Erro) ---
  Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Fc = Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Zero();

  // Bloco Orientação (Theta)
  Fc.block<3, 3>(ErrorIdx::THETAX, ErrorIdx::THETAX) = -skew(w_hat);
  Fc.block<3, 3>(ErrorIdx::THETAX, ErrorIdx::BGX)    = -Eigen::Matrix3d::Identity();  // Ou -R_body_to_world dependendo da definição do erro

  // Bloco Velocidade
  Fc.block<3, 3>(ErrorIdx::VX, ErrorIdx::THETAX) = -R_body_to_world * skew(a_hat);  // Cross product effect
  Fc.block<3, 3>(ErrorIdx::VX, ErrorIdx::BAX)    = -R_body_to_world;

  // Bloco Posição
  Fc.block<3, 3>(ErrorIdx::PX, ErrorIdx::VX) = Eigen::Matrix3d::Identity();

  // --- 5. Matriz de Ruído G_c ---
  Eigen::Matrix<double, DIM_ERROR, 12> Gc = Eigen::Matrix<double, DIM_ERROR, 12>::Zero();

  Gc.block<3, 3>(ErrorIdx::THETAX, 0) = -Eigen::Matrix3d::Identity();  // n_g
  Gc.block<3, 3>(ErrorIdx::BGX, 3)    = Eigen::Matrix3d::Identity();   // n_wg
  Gc.block<3, 3>(ErrorIdx::VX, 6)     = -R_body_to_world;              // n_a
  Gc.block<3, 3>(ErrorIdx::BAX, 9)    = Eigen::Matrix3d::Identity();   // n_wa

  // --- 6. Matriz de Covariância do Ruído Q_c ---
  Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block<3, 3>(0, 0)             = Eigen::Matrix3d::Identity() * noise_gyr_ * noise_gyr_;
  Qc.block<3, 3>(3, 3)             = Eigen::Matrix3d::Identity() * noise_bias_gyr_ * noise_bias_gyr_;
  Qc.block<3, 3>(6, 6)             = Eigen::Matrix3d::Identity() * noise_acc_ * noise_acc_;
  Qc.block<3, 3>(9, 9)             = Eigen::Matrix3d::Identity() * noise_bias_acc_ * noise_bias_acc_;

  // --- 7. Discretização e Update da Covariância ---
  // P_k+1 = Phi * P_k * Phi' + Qd
  Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Phi = Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity() + Fc * dt;
  Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Qd  = (Gc * Qc * Gc.transpose()) * dt;

  cov_ = Phi * cov_ * Phi.transpose() + Qd;

  // Simetria e estabilidade numérica
  cov_ = 0.5 * (cov_ + cov_.transpose());
}

// --- SETTERS (ESSENCIAIS PARA O RESET) ---

void ImuPropagator::set_state(const Eigen::Matrix<double, DIM_STATE, 1> &state) {
  state_ = state;
}

void ImuPropagator::set_state(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Quaterniond &q) {
  // Atualiza apenas a parte cinemática, mantendo os biases que o IMU já estimou (se houver)
  state_.segment<3>(StateIMU::PX) = p;
  state_.segment<3>(StateIMU::VX) = v;

  // Normalizar quaternião recebido
  Eigen::Quaterniond q_norm = q.normalized();
  state_(StateIMU::QW)      = q_norm.w();
  state_(StateIMU::QX)      = q_norm.x();
  state_(StateIMU::QY)      = q_norm.y();
  state_(StateIMU::QZ)      = q_norm.z();

  std::cout << "IMU Propagator state set to:" << std::endl;
  std::cout << "  Position:    " << p.transpose() << std::endl;
  std::cout << "  Velocity:    " << v.transpose() << std::endl;
  std::cout << "  Orientation: " << q_norm.coeffs().transpose() << std::endl;
}

void ImuPropagator::set_covariance(const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> &cov) {
  cov_ = cov;
}

void ImuPropagator::set_noise_coefficients(double noise_gyr, double noise_acc, double noise_bias_gyr, double noise_bias_acc) {
  noise_gyr_      = noise_gyr;
  noise_acc_      = noise_acc;
  noise_bias_gyr_ = noise_bias_gyr;
  noise_bias_acc_ = noise_bias_acc;
}

// --- GETTERS PRINCIPAIS ---

const Eigen::Matrix<double, DIM_STATE, 1> &ImuPropagator::get_state() const {
  return state_;
}

const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> &ImuPropagator::get_covariance() const {
  return cov_;
}

// --- GETTERS AUXILIARES (HELPERS) ---

Eigen::Quaterniond ImuPropagator::get_orientation() const {
  return Eigen::Quaterniond(state_(StateIMU::QW), state_(StateIMU::QX), state_(StateIMU::QY), state_(StateIMU::QZ));
}

Eigen::Vector3d ImuPropagator::get_velocity() const {
  return state_.segment<3>(StateIMU::VX);
}

Eigen::Vector3d ImuPropagator::get_position() const {
  return state_.segment<3>(StateIMU::PX);
}

Eigen::Vector3d ImuPropagator::get_bias_acc() const {
  return state_.segment<3>(StateIMU::BAX);
}

Eigen::Vector3d ImuPropagator::get_bias_gyr() const {
  return state_.segment<3>(StateIMU::BGX);
}

// --- UTILS ---

Eigen::Matrix3d ImuPropagator::skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

}  // namespace laser_uav_lib