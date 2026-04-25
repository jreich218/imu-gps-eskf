#ifndef ESKF_HPP
#define ESKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>

#include "scene_types.hpp"

/// Diagnostics from one 2D GPS update.
struct GpsUpdateResult {
    Eigen::Vector2d innovation_xy{0.0,
                                  0.0};  ///< Horizontal innovation in meters.
    double nis = 0.0;                    ///< Normalized innovation squared.
};

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
     * @brief Propagate the filter from the previous IMU sample to the current
     * one.
     *
     * @param current_imu_sample Current IMU sample k used for propagation from
     *     t_{k-1} to t_k.
     */
    void Predict(const ImuSample& current_imu_sample);

    /**
     * @brief Update the filter with one 2D GPS position measurement.
     *
     * @param gps_sample GPS measurement at the current update time.
     * @return Innovation and NIS from this GPS update.
     */
    GpsUpdateResult UpdateGps(const GpsSample& gps_sample);

   private:
    /// Nominal state at the latest propagated IMU timestamp.
    ///
    /// At Eskf::Predict entry, this is the previous state at t_{k-1}. At
    /// Eskf::Predict exit, this has been advanced to the current state at t_k.
    struct NominalState {
        Eigen::Vector3d p_G{0.0, 0.0, 0.0};  ///< Nominal position in frame G.
        Eigen::Vector3d v_G{0.0, 0.0, 0.0};  ///< Nominal velocity in frame G.
        Eigen::Quaterniond q_GI{
            1.0, 0.0, 0.0, 0.0};  ///< Nominal attitude quaternion from the
                                  ///< state's IMU frame to G.
    };

    NominalState nominal_state_;
    Eigen::Matrix<double, 9, 9> P_ = Eigen::Matrix<double, 9, 9>::Zero();
    std::int64_t previous_imu_utime_ = 0;
    bool initialized_ = false;
};

#endif  // ESKF_HPP
