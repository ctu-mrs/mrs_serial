/* some code copied from MAVLInk_DroneLights by Juan Pedro López */

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
#include "mavlink/mavlink.h"
#include "SBGC_lib/SBGC.h"
#include "serial_port.h"
#include "mrs_serial/GimbalParamsConfig.h"

namespace gimbal {

    using mat3_t = Eigen::Matrix3d;
    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;
    using vec3_t = Eigen::Vector3d;

    using Config = mrs_serial::GimbalParamsConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    /* class Gimbal //{ */

    class Gimbal : public nodelet::Nodelet {
    private:
        /* enums and struct defines //{ */

        static constexpr char EULER_ORDER_PARAM_ID[16] = "G_EULER_ORDER\0";

        enum class euler_order_t {
            pitch_roll_yaw = 0,
            roll_pitch_yaw = 1,
            pitchmotor_roll_yawmotor = 2,
            roll_pitchmotor_yawmotor = 3,
            yaw_roll_pitch = 4,
            unknown,
        };

        enum class angle_input_mode_t {
            angle_body_frame = 0,
            angular_rate = 1,
            angle_absolute_frame = 2,
        };

        /* struct mount_config_t //{ */
        struct mount_config_t {
            bool stabilize_roll;
            bool stabilize_pitch;
            bool stabilize_yaw;
            angle_input_mode_t roll_input_mode;
            angle_input_mode_t pitch_input_mode;
            angle_input_mode_t yaw_input_mode;

            bool from_rosparam(ros::NodeHandle &nh) {
                mrs_lib::ParamLoader pl(nh);
                pl.loadParam("mount/stabilize_roll", stabilize_roll);
                pl.loadParam("mount/stabilize_pitch", stabilize_pitch);
                pl.loadParam("mount/stabilize_yaw", stabilize_yaw);

                const int roll_mode = pl.loadParam2<int>("mount/roll_input_mode");
                const int pitch_mode = pl.loadParam2<int>("mount/pitch_input_mode");
                const int yaw_mode = pl.loadParam2<int>("mount/yaw_input_mode");

                bool ret = pl.loadedSuccessfully();

                if (roll_mode >= 0 && roll_mode < 3)
                    roll_input_mode = static_cast<angle_input_mode_t>(roll_mode);
                else {
                    ROS_ERROR(
                            "[Gimbal]: Invalid value of roll input mode (got %d)!\nValid values are:\n\t0\t(angle body frame)\n\t1\t(angular rate)\n\t2\t(angle absolute "
                            "frame)",
                            roll_mode);
                    ret = false;
                }

                if (pitch_mode >= 0 && pitch_mode < 3)
                    pitch_input_mode = static_cast<angle_input_mode_t>(pitch_mode);
                else {
                    ROS_ERROR(
                            "[Gimbal]: Invalid value of pitch input mode (got %d)!\nValid values are:\n\t0\t(angle body frame)\n\t1\t(angular rate)\n\t2\t(angle absolute "
                            "frame)",
                            roll_mode);
                    ret = false;
                }

                if (yaw_mode >= 0 && yaw_mode < 3)
                    yaw_input_mode = static_cast<angle_input_mode_t>(yaw_mode);
                else {
                    ROS_ERROR(
                            "[Gimbal]: Invalid value of yaw input mode (got %d)!\nValid values are:\n\t0\t(angle body frame)\n\t1\t(angular rate)\n\t2\t(angle absolute "
                            "frame)",
                            roll_mode);
                    ret = false;
                }

                return ret;
            }
        } m_mount_config;
        //}

        // https://mavlink.io/en/messages/common.html

        enum MAV_CMD {
            MAV_CMD_DO_MOUNT_CONFIGURE = 204,  //< Mission command to configure a camera or antenna mount
            MAV_CMD_DO_MOUNT_CONTROL = 205,    //< Mission command to control a camera or antenna mount
        };

        enum MAV_MOUNT_MODE {
            MAV_MOUNT_MODE_RETRACT = 0,            //<	Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
            MAV_MOUNT_MODE_NEUTRAL = 1,            //<	Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
            MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,  //<	Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
            MAV_MOUNT_MODE_RC_TARGETING = 3,       //<	Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
            MAV_MOUNT_MODE_GPS_POINT = 4,          //<	Load neutral position and start to point to Lat,Lon,Alt
            MAV_MOUNT_MODE_SYSID_TARGET = 5,       //<	Gimbal tracks system with specified system ID
            MAV_MOUNT_MODE_HOME_LOCATION = 6,      //<	Gimbal tracks home location
        };

        //}

        template<typename T>
        T rad2deg(const T x) { return x / M_PI * 180; }

        bool connect();

        /* static constexpr uint32_t m_request_data_flags = cmd_realtime_data_custom_flags_target_angles | cmd_realtime_data_custom_flags_target_speed | cmd_realtime_data_custom_flags_stator_rotor_angle | cmd_realtime_data_custom_flags_encoder_raw24; */
        static constexpr uint32_t m_request_data_flags =
                cmd_realtime_data_custom_flags_z_vector_h_vector | cmd_realtime_data_custom_flags_stator_rotor_angle;
        static constexpr double units2rads = 0.02197265625 / 180.0 * M_PI;

        static constexpr int ROLL_IDX = 0;
        static constexpr int PITCH_IDX = 1;
        static constexpr int YAW_IDX = 2;

        void sending_loop([[maybe_unused]] const ros::TimerEvent &evt);

        bool request_data(uint32_t request_data_flags);

        void attitude_cbk(const nav_msgs::Odometry::ConstPtr &odometry_in);

        void cmd_orientation_cbk(const geometry_msgs::QuaternionStamped::ConstPtr &cmd_orientation);

        void cmd_pry_cbk(const mrs_msgs::GimbalPRY::ConstPtr &cmd_pry);

        bool command_mount(const double pitch, const double roll, const double yaw);

        void receiving_loop([[maybe_unused]] const ros::TimerEvent &evt);

        void process_param_value_msg(const mavlink_param_value_t &param_value);

        void process_custom_data_msg(const SBGC_cmd_realtime_data_custom_t &data);

        quat_t pry2quat(double pitch, double roll, double yaw);

        void callbackDynamicReconfigure(mrs_serial::GimbalParamsConfig &config, uint32_t level) {
            ROS_INFO("[Gimbal] dynamic_reconf entered");
            if (config.motors_on_off == false) {
                stop_gimbal_motors();
            } else {
                start_gimbal_motors();
            }
            turn_camera_on_deg(
                    static_cast<int16_t>(config.pitch_angle), static_cast<int16_t>(config.yaw_angle));
        }

        void start_gimbal_motors();

        void stop_gimbal_motors();

        bool turn_camera_on_deg(int16_t pitch_deg, int16_t yaw_deg, int16_t pitch_speed = 737, int16_t yaw_speed = 737);


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
        ros::Publisher m_pub_command;

        tf2_ros::TransformBroadcaster m_pub_transform;

        mrs_lib::Transformer m_transformer;
        serial_port::SerialPort m_serial_port;

        // --------------------------------------------------------------
        // |                 Parameters, loaded from ROS                |
        // --------------------------------------------------------------
        std::string m_uav_name;
        std::string m_portname;
        int m_baudrate;
        ros::Duration m_heartbeat_period;

        int m_driver_system_id;
        int m_driver_component_id;

        int m_gimbal_system_id;
        int m_gimbal_component_id;

        std::vector<int> m_stream_request_ids;
        std::vector<int> m_stream_request_rates;

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

        euler_order_t m_euler_ordering = euler_order_t::unknown;

    public:
//        FOR DYNAMIC PARAMS
        boost::recursive_mutex m_config_mutex;
        boost::shared_ptr<ReconfigureServer> reconfigure_server;

        /* onInit() //{ */
        void onInit() override;
        //}

        SBGC_Parser sbgc_parser;

    };

    //}

}  // namespace gimbal