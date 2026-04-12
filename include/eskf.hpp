#ifndef ESKF_HPP
#define ESKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "scene_types.hpp"

/// Error-state Kalman filter over position, velocity, and attitude.
class Eskf {
   public:
    /**
     * @brief Initialize the filter from the startup initialization.
     *
     * @param startup_initialization Startup state used to begin filtering.
     */
    void Initialize(const StartupInitialization& startup_initialization);

    /**
     * @brief Propagate the filter with one IMU sample.
     *
     * @param imu_sample IMU sample used for propagation.
     */
    void Predict(const ImuSample& imu_sample);

   private:
    struct NominalState {
        Eigen::Vector3d p_G{0.0, 0.0, 0.0};
        Eigen::Vector3d v_G{0.0, 0.0, 0.0};
        Eigen::Quaterniond q_GI{1.0, 0.0, 0.0, 0.0};
    };

    NominalState x_;
    Eigen::Matrix<double, 9, 9> P_ = Eigen::Matrix<double, 9, 9>::Zero();
    std::int64_t last_imu_utime_ = 0;
    bool initialized_ = false;
};

#endif  // ESKF_HPP
