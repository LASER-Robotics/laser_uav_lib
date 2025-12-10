#ifndef LASER_UAV_LIB_IMU_PROPAGATOR_HPP
#define LASER_UAV_LIB_IMU_PROPAGATOR_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>  // Necessário para Quaterniond

namespace laser_uav_lib
{

// Dimensões exatas conforme o artigo (sem feature)
static const int DIM_STATE = 16;  // q(4) + bg(3) + v(3) + ba(3) + p(3)
static const int DIM_ERROR = 15;  // th(3) + bg(3) + v(3) + ba(3) + p(3)

// Ordem baseada na Eq. (1) do artigo
namespace StateIMU
{
enum
{
  // 1. Orientação (Quaternion)
  QW = 0,
  QX = 1,
  QY = 2,
  QZ = 3,

  // 2. Bias Giroscópio
  BGX = 4,
  BGY = 5,
  BGZ = 6,

  // 3. Velocidade
  VX = 7,
  VY = 8,
  VZ = 9,

  // 4. Bias Acelerómetro
  BAX = 10,
  BAY = 11,
  BAZ = 12,

  // 5. Posição
  PX = 13,
  PY = 14,
  PZ = 15
};
}

// Ordem baseada na Eq. (5) do artigo
namespace ErrorIdx
{
enum
{
  // 1. Erro de Orientação (Theta)
  THETAX = 0,
  THETAY = 1,
  THETAZ = 2,

  // 2. Erro Bias Giroscópio
  BGX = 3,
  BGY = 4,
  BGZ = 5,

  // 3. Erro Velocidade
  VX = 6,
  VY = 7,
  VZ = 8,

  // 4. Erro Bias Acelerómetro
  BAX = 9,
  BAY = 10,
  BAZ = 11,

  // 5. Erro Posição
  PX = 12,
  PY = 13,
  PZ = 14
};
}

class ImuPropagator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuPropagator();
  ~ImuPropagator();

  // Método principal de predição
  void propagate(const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &linear_acceleration, double dt);

  // --- SETTERS (Para Reset/Sincronização) ---

  /**
   * @brief Define o estado completo diretamente (vetor 16x1)
   */
  void set_state(const Eigen::Matrix<double, DIM_STATE, 1> &state);

  /**
   * @brief Define apenas a parte cinemática (usado no reset do EKF)
   * Mantém os biases atuais inalterados.
   */
  void set_state(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Quaterniond &q);

  /**
   * @brief Define a covariância (opcional, para resetar incerteza)
   */
  void set_covariance(const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> &cov);

  /**
   * @brief Configura os parâmetros de ruído do IMU
   */
  void set_noise_coefficients(double noise_gyr, double noise_acc, double noise_bias_gyr, double noise_bias_acc);


  // --- GETTERS (Para uso no processImuMeasurement) ---

  const Eigen::Matrix<double, DIM_STATE, 1>         &get_state() const;
  const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> &get_covariance() const;

  // Helpers semânticos para facilitar a leitura no EKF
  Eigen::Quaterniond get_orientation() const;
  Eigen::Vector3d    get_velocity() const;
  Eigen::Vector3d    get_position() const;
  Eigen::Vector3d    get_bias_acc() const;
  Eigen::Vector3d    get_bias_gyr() const;

private:
  Eigen::Matrix<double, DIM_STATE, 1>         state_;
  Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> cov_;

  // Parâmetros de Ruído (Noise Density & Random Walk)
  double noise_gyr_;       // rad/s / sqrt(Hz)
  double noise_acc_;       // m/s^2 / sqrt(Hz)
  double noise_bias_gyr_;  // rad/s^2 / sqrt(Hz)
  double noise_bias_acc_;  // m/s^3 / sqrt(Hz)

  Eigen::Vector3d gravity_;

  Eigen::Matrix3d skew(const Eigen::Vector3d &v);
};

}  // namespace laser_uav_lib
#endif  // LASER_UAV_LIB_IMU_PROPAGATOR_HPP