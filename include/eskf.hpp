#ifndef ESKF_HPP
#define ESKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>

#include "scene_types.hpp"

class Eskf {
   public:
    void Initialize(const StartupInitialization& startup_initialization);

   private:
    struct NominalState {
        Eigen::Vector3d p_G{0.0, 0.0, 0.0};
        Eigen::Vector3d v_G{0.0, 0.0, 0.0};
        Eigen::Quaterniond q_GI{1.0, 0.0, 0.0, 0.0};
    };

    NominalState x_;
    Eigen::Matrix<double, 9, 9> P_;
    std::atomic_int64_t last_imu_utime_ = 0;
    bool initialized_ = false;
};

#endif  // ESKF_HPP