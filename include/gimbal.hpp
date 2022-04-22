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
#include <mrs_lib/attitude_converter.h>

#include <limits>
#include <mutex>
#include <string>

#include <mrs_msgs/GimbalPRY.h>
#include <dynamic_reconfigure/server.h>
#include "SBGC_lib/SBGC.h"
#include "serial_port.h"
#include <mrs_serial/gimbalConfig.h>

#include <tf2_eigen/tf2_eigen.h>

namespace gimbal {

    using mat3_t = Eigen::Matrix3d;
    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;
    using vec3_t = Eigen::Vector3d;

    using Config = mrs_serial::gimbalConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    /* class Gimbal //{ */

    class Gimbal : public nodelet::Nodelet {
    private:
        /* enums and struct defines //{ */

        enum class euler_order_t {
            pitch_roll_yaw = 0,
            roll_pitch_yaw = 1,
            pitchmotor_roll_yawmotor = 2,
            roll_pitchmotor_yawmotor = 3,
            yaw_roll_pitch = 4,
            yaw_pitch_roll = 5,
            unknown,
        };

        //}

        template<typename T>
        T rad2deg(const T x) { return x / M_PI * 180; }

        template<typename T>
        T deg2rad(const T x) { return x * M_PI / 180; }

        template<typename T>
        bool eq(T a1, T a2) { return abs(a1 - a2) <= std::numeric_limits<double>::epsilon(); }

        bool connect();

        bool m_correct_euler_order = false;

        /* static constexpr uint32_t m_request_data_flags = cmd_realtime_data_custom_flags_target_angles | cmd_realtime_data_custom_flags_target_speed | cmd_realtime_data_custom_flags_stator_rotor_angle | cmd_realtime_data_custom_flags_encoder_raw24; */
        static constexpr uint32_t
                m_request_data_flags =
                cmd_realtime_data_custom_flags_z_vector_h_vector |
                cmd_realtime_data_custom_flags_stator_rotor_angle |
                cmd_realtime_data_custom_flags_target_speed;
        static constexpr double units2rads = 0.02197265625 / 180.0 * M_PI;

        static constexpr int ROLL_IDX = 0;
        static constexpr int PITCH_IDX = 1;
        static constexpr int YAW_IDX = 2;

        void sending_loop([[maybe_unused]] const ros::TimerEvent &evt);

        bool request_data(uint32_t request_data_flags);

        void attitude_cbk(const nav_msgs::Odometry::ConstPtr &odometry_in);

        void cmd_orientation_cbk(const geometry_msgs::QuaternionStamped::ConstPtr &cmd_orientation);

        void cmd_pry_cbk(const mrs_msgs::GimbalPRY::ConstPtr &cmd_pry);

        bool rotate_gimbal_PRY(double pitch, double roll, double yaw);

        void receiving_loop([[maybe_unused]] const ros::TimerEvent &evt);

        void process_custom_data_msg(const SBGC_cmd_realtime_data_custom_t &data);

        quat_t pry2quat(double pitch, [[maybe_unused]] double roll, double yaw);

        vec3_t rotation2rpy(const mat3_t &R);

        void rotate_gimbal_PRY_between_frames(const double &pitch, const double &roll, const double &yaw,
                                              const std::string &in_frame_id, const std::string &out_frame_id);

        void rotate_gimbal_PRY_rot_mat(double pitch, double roll, double yaw, const mat3_t &rot_mat);

        void callbackDynamicReconfigure(const mrs_serial::gimbalConfig &config, const uint32_t level) {
            ROS_INFO("[Gimbal] dynamic_reconf entered");
            if (config.motors_on_off == false) {
                stop_gimbal_motors();
            } else {
                start_gimbal_motors();
            }
            const double pitch = deg2rad(static_cast<double>(config.pitch_angle));
            const double roll = deg2rad(static_cast<double>(config.roll_angle));
            const double yaw = deg2rad(static_cast<double>(config.yaw_angle));

            m_speed_yaw = config.yaw_speed * 0.1220740379;
            m_speed_pitch = config.pitch_speed * 0.1220740379;
            m_speed_roll = config.roll_speed * 0.1220740379;
            m_interval = config.interval;

            rotate_gimbal_PRY_between_frames(pitch, roll, yaw, m_base_frame_id, m_stabilization_frame_id);

            //            ROS_INFO_THROTTLE(1.0,
            //                              "[Gimbal]: |dynparam reconf| Sending mount control command\n\t\tpitch: %.0fdeg\n\t\troll: %.0fdeg\n\t\tyaw: %.0fdeg).",
            //                              rad2deg(pitch), rad2deg(roll), rad2deg(yaw));
        }

        void start_gimbal_motors();

        void stop_gimbal_motors();

        // --------------------------------------------------------------
        // |                    ROS-related variables                   |
        // --------------------------------------------------------------
        ros::NodeHandle m_nh;
        ros::Timer m_tim_sending;
        ros::Timer m_tim_receiving;

        ros::Subscriber m_sub_attitude;
        ros::Subscriber m_sub_command;
        ros::Subscriber m_sub_pry;

        ros::Publisher m_pub_attitude;
        ros::Publisher m_pub_speed;
        ros::Publisher m_pub_command;
        ros::Publisher m_pub_orientation_pry;

        tf2_ros::TransformBroadcaster m_pub_transform;

        std::unique_ptr<mrs_lib::Transformer> m_transformer;
        serial_port::SerialPort m_serial_port;

        // --------------------------------------------------------------
        // |                 Parameters, loaded from ROS                |
        // --------------------------------------------------------------
        std::string m_uav_name;
        std::string m_portname;
        int m_baudrate;
        ros::Duration m_heartbeat_period;

        std::string m_stabilization_frame_id;
        std::string m_base_frame_id;
        std::string m_stabilized_frame_id;

        // --------------------------------------------------------------
        // |                   Other member variables                   |
        // --------------------------------------------------------------
        const ros::Time m_start_time = ros::Time::now();
        ros::Time m_last_sent_time = ros::Time::now();
        bool m_is_connected = false;

        int m_hbs_since_last_request = 2;
        int m_hbs_request_period = 5;

        int m_hbs_since_last_configure = 3;
        int m_hbs_configure_period = 5;

        int m_hbs_since_last_param_request = 4;
        int m_hbs_param_request_period = 5;

        size_t m_msgs_received = 0;
        size_t m_valid_msgs_received = 0;

        double m_speed_yaw = 0;
        double m_speed_pitch = 0;
        double m_speed_roll = 0;

        int16_t m_interval;

    public:
//        FOR DYNAMIC PARAMS
        std::shared_ptr<ReconfigureServer> reconfigure_server;

        /* onInit() //{ */
        void onInit() override;
        //}

        SBGC_Parser sbgc_parser;

    };

    //}

}  // namespace gimbal
