#pragma once

#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <limits>
#include <mutex>
#include <string>

#include <tf2_eigen/tf2_eigen.h>

namespace gimbal {

    using mat3_t = Eigen::Matrix3d;
    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;
    using vec3_t = Eigen::Vector3d;

    /* class Gimbal //{ */

    class Gimbal : public nodelet::Nodelet {
    public:

        void onInit() override;

    private:
        ros::NodeHandle m_nh;

        int m_driver_system_id;
        int m_driver_component_id;

        int m_gimbal_system_id = 0;
        int m_gimbal_component_id = 1;

        static constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
    };

    //}

}  // namespace gimbal

