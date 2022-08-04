/* some code copied from MAVLInk_DroneLights by Juan Pedro López */

#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>

#include <limits>
#include <mutex>
#include <string>

#include <mrs_msgs/GimbalPRY.h>
#include <mavconn/interface.h>
#include <mavconn/udp.h>

#include <mavlink/v2.0/common/mavlink_msg_heartbeat.hpp>
#include <mavlink/v2.0/common/mavlink_msg_message_interval.hpp>
#include <mavlink/v2.0/common/mavlink_msg_param_request_list.hpp>
#include <mavlink/v2.0/common/mavlink_msg_param_request_read.hpp>
#include <mavlink/v2.0/common/mavlink_msg_param_set.hpp>
#include <mavlink/v2.0/common/mavlink_msg_attitude.hpp>
#include <mavlink/v2.0/common/mavlink_msg_global_position_int.hpp>

/* using mavlink_message_t = mavlink::mavlink_message_t; */
/* #include <mavlink/v2.0/common/mavlink_msg_sys_status.hpp> */
/* #include <mavlink/v2.0/common/mavlink_msg_param_value.h> */
/* #include <mavlink/v2.0/common/mavlink_msg_attitude.h> */

#include <mavlink/v2.0/common/mavlink_msg_command_ack.hpp>
#include <mavlink/v2.0/common/mavlink_msg_command_long.hpp>
#include <mavlink/v2.0/common/mavlink_msg_gimbal_device_set_attitude.hpp>
#include <mavlink/v2.0/common/mavlink_msg_gimbal_manager_set_attitude.hpp>
#include <mavlink/v2.0/common/mavlink_msg_v2_extension.hpp>

namespace gimbal
{

  /* class Gimbal //{ */

  class Gimbal : public nodelet::Nodelet
  {
  private:
    /* enums and struct defines //{ */

    enum class angle_input_mode_t
    {
      angle_body_frame = 0,
      angular_rate = 1,
      angle_absolute_frame = 2,
    };

    /* struct mount_config_t //{ */

    struct mount_config_t
    {
      bool stabilize_roll;
      bool stabilize_pitch;
      bool stabilize_yaw;
      angle_input_mode_t roll_input_mode;
      angle_input_mode_t pitch_input_mode;
      angle_input_mode_t yaw_input_mode;

      bool from_rosparam(ros::NodeHandle& nh)
      {
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
        else
        {
          ROS_ERROR(
              "[Gimbal]: Invalid value of roll input mode (got %d)!\nValid values are:\n\t0\t(angle body frame)\n\t1\t(angular rate)\n\t2\t(angle absolute "
              "frame)",
              roll_mode);
          ret = false;
        }

        if (pitch_mode >= 0 && pitch_mode < 3)
          pitch_input_mode = static_cast<angle_input_mode_t>(pitch_mode);
        else
        {
          ROS_ERROR(
              "[Gimbal]: Invalid value of pitch input mode (got %d)!\nValid values are:\n\t0\t(angle body frame)\n\t1\t(angular rate)\n\t2\t(angle absolute "
              "frame)",
              roll_mode);
          ret = false;
        }

        if (yaw_mode >= 0 && yaw_mode < 3)
          yaw_input_mode = static_cast<angle_input_mode_t>(yaw_mode);
        else
        {
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

    struct awaiting_ack_t
    {
      mavlink::common::msg::COMMAND_LONG msg;
      ros::Time last_try;
      int tries = 0;

      struct hash
      {
        size_t operator()(const awaiting_ack_t& el)
        {
          return std::hash<uint16_t>()(el.msg.command);
        }
      };
    };

    enum class OS_Cmd_t
    {
      set_system_mode = 0,
      take_snapshot = 1,
      set_rec_state = 2,
      set_sensor = 3,
      set_fov = 4,
      set_sharpness = 5,
      // others
      stream_control = 57,
    };

    enum class camera_mode_t
    {
      stow = 0,
      pilot = 1,
      hold_coordinate = 2,
      observation = 3,
      local_position = 4,
      global_position = 5,
      grr = 6,
      tracking = 7,
      epr = 8,
      nadir = 9,
      nadir_scan = 10,
      scan_2D = 11,
      point_to_coordinate = 12,
      unstabilized_position = 13,
    };

    enum class report_type_t : uint16_t
    {
      system = 0,
      los = 1,
      ground_crossing = 2,
      sd_card = 6,
      snapshot = 5,
      video = 7,
      los_direction_and_rate = 8,
      object_detection = 9,
      object_detection_car_count = 16,
      imu = 10,
      fire = 11,
      tracking = 12,
      lpr = 13,
      augmented_reality_markers = 14,
      parameter = 15,
      oglr = 17,
      video_motion_detection = 18,
    };

    std::string to_str(const report_type_t report_type)
    {
      switch (report_type)
      {
        case report_type_t::system: return "system";
        case report_type_t::los: return "los";
        case report_type_t::ground_crossing: return "ground_crossing";
        case report_type_t::sd_card: return "sd_card";
        case report_type_t::snapshot: return "snapshot";
        case report_type_t::video: return "video";
        case report_type_t::los_direction_and_rate: return "los_direction_and_rate";
        case report_type_t::object_detection: return "object_detection";
        case report_type_t::object_detection_car_count: return "object_detection_car_count";
        case report_type_t::imu: return "imu";
        case report_type_t::fire: return "fire";
        case report_type_t::tracking: return "tracking";
        case report_type_t::lpr: return "lpr";
        case report_type_t::augmented_reality_markers: return "augmented_reality_markers";
        case report_type_t::parameter: return "parameter";
        case report_type_t::oglr: return "oglr";
        case report_type_t::video_motion_detection: return "video_motion_detection";
        default: return "unknown";
      }
    }

    enum class system_state_t : uint8_t
    {
      stow = 0,
      pilot = 1,
      retract = 2,
      retract_lock = 3,
      observation = 4,
      grr = 5,
      hold_coordinate = 6,
      point_to_coordinate = 7,
      local_position = 8,
      global_position = 9,
      track = 10,
      extended_pitch_range = 11,
      bit = 12,
      nadir = 13,
    };

    std::string to_str(const system_state_t system_state)
    {
      switch (system_state)
      {
        case system_state_t::stow: return "stow";
        case system_state_t::pilot: return "pilot";
        case system_state_t::retract: return "retract";
        case system_state_t::retract_lock: return "retract_lock";
        case system_state_t::observation: return "observation";
        case system_state_t::grr: return "grr";
        case system_state_t::hold_coordinate: return "hold_coordinate";
        case system_state_t::point_to_coordinate: return "point_to_coordinate";
        case system_state_t::local_position: return "local_position";
        case system_state_t::global_position: return "global_position";
        case system_state_t::track: return "track";
        case system_state_t::extended_pitch_range: return "extended_pitch_range";
        case system_state_t::bit: return "bit";
        case system_state_t::nadir: return "nadir";
        default: return "unknown";
      }
    }

    enum class stream_command_t : uint8_t
    {
      set_stream_mode = 0,
      set_pip_mode = 1,
      set_sbs_mode = 2,
    };

    enum class stream_mode_t : uint8_t
    {
      disabled = 0,
      day = 1,
      ir = 2,
      fusion = 3,
      pip = 4,
      side_by_side = 5,
    };

    std::string to_str(const stream_mode_t stream_mode)
    {
      switch (stream_mode)
      {
        case stream_mode_t::disabled: return "disabled";
        case stream_mode_t::day: return "day";
        case stream_mode_t::ir: return "ir";
        case stream_mode_t::fusion: return "fusion";
        case stream_mode_t::pip: return "pip";
        case stream_mode_t::side_by_side: return "side_by_side";
        default: return "unknown";
      }
    }

    enum class pip_mode_t : uint8_t
    {
      pip_day_large = 0,
      pip_ir_large = 1,
    };

    enum class sbs_mode_t : uint8_t
    {
      day_left_ir_right = 0,
      day_right_ir_left = 1,
    };

    using mat3_t = Eigen::Matrix3d;
    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;
    using vec3_t = Eigen::Vector3d;

    //}

    template <typename T>
    T rad2deg(const T x) {return x/M_PI*180;}

  public:
    /* onInit() //{ */

    virtual void onInit() override
    {

      // Get paramters
      m_nh = ros::NodeHandle("~");

      ros::Time::waitForValid();

      mrs_lib::ParamLoader pl(m_nh);

      pl.loadParam("uav_name", m_uav_name);
      const auto bind_host = pl.loadParam2<std::string>("bind/host");
      const auto bind_port = pl.loadParam2<int>("bind/port");
      const auto remote_host = pl.loadParam2<std::string>("remote/host");
      const auto remote_port = pl.loadParam2<int>("remote/port");
      pl.loadParam("base_frame_id", m_base_frame_id);
      pl.loadParam("child_frame_id", m_child_frame_id);

      const auto heartbeat_period = pl.loadParam2<ros::Duration>("mavlink/heartbeat_period");
      const auto command_period = pl.loadParam2<ros::Duration>("mavlink/command_period");
      pl.loadParam("mavlink/resend/duration", m_resend_duration);
      pl.loadParam("mavlink/resend/max_tries", m_resend_max_tries);
      pl.loadParam("mavlink/driver/system_id", m_driver_system_id);
      pl.loadParam("mavlink/driver/component_id", m_driver_component_id);
      pl.loadParam("mavlink/gimbal/system_id", m_gimbal_system_id);
      pl.loadParam("mavlink/gimbal/component_id", m_gimbal_component_id);

      pl.loadParam("stream_requests/ids", m_stream_request_ids);
      pl.loadParam("stream_requests/rates", m_stream_request_rates);

      if (m_stream_request_ids.size() != m_stream_request_rates.size())
      {
        ROS_ERROR("[Gimbal]: Number of requested stream IDs has to be the same as the number of rates! Ending.");
        ros::shutdown();
        return;
      }

      if (!m_mount_config.from_rosparam(m_nh))
      {
        ROS_ERROR("[Gimbal]: Failed to load mount configuration parameters! Ending.");
        ros::shutdown();
        return;
      }

      if (!pl.loadedSuccessfully())
      {
        ROS_ERROR("[Gimbal]: Some compulsory parameters could not be loaded! Ending.");
        ros::shutdown();
        return;
      }

      // Output loaded parameters to console for double checking
      ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
      ROS_INFO_THROTTLE(1.0, "[%s] bind:   %s:%d", ros::this_node::getName().c_str(), bind_host.c_str(), bind_port);
      ROS_INFO_THROTTLE(1.0, "[%s] remote: %s:%d", ros::this_node::getName().c_str(), remote_host.c_str(), remote_port);

      const bool connected = connect(m_driver_system_id, m_driver_component_id, bind_host, bind_port, remote_host, remote_port);
      if (connected)
      {
        m_tim_heartbeat = m_nh.createTimer(heartbeat_period, &Gimbal::heartbeat_loop, this);
        m_tim_command = m_nh.createTimer(command_period, &Gimbal::command_loop, this);

        m_pub_attitude = m_nh.advertise<nav_msgs::Odometry>("attitude_out", 10);
        m_pub_command = m_nh.advertise<nav_msgs::Odometry>("current_setpoint", 10);

        m_sub_attitude = m_nh.subscribe("attitude_in", 10, &Gimbal::attitude_cbk, this);
        m_sub_command = m_nh.subscribe("cmd_orientation", 10, &Gimbal::cmd_orientation_cbk, this);
        m_sub_pry = m_nh.subscribe("cmd_pry", 10, &Gimbal::cmd_pry_cbk, this);
        m_sub_stream_mode = m_nh.subscribe("stream_mode", 10, &Gimbal::stream_mode_cbk, this);

        m_transformer = mrs_lib::Transformer("Gimbal");
        m_transformer.setLookupTimeout(ros::Duration(0.1));
      } else
      {
        ROS_ERROR("[Gimbal]: Could not connect to the serial port! Ending.");
        ros::shutdown();
        return;
      }
    }
    //}

  private:
    /* connect() //{ */

    bool connect(const uint8_t driver_system_id, const uint8_t driver_component_id, const std::string& bind_host, const unsigned short bind_port, const std::string& remote_host, const unsigned short remote_port)
    {
      ROS_INFO_THROTTLE(1.0, "[%s]: Openning the UDP connection.", ros::this_node::getName().c_str());

      try
      {
        std::scoped_lock lck(m_mavconn_mtx);
        const mavconn::MAVConnInterface::ReceivedCb cbk = std::bind(&Gimbal::msg_callback, this, std::placeholders::_1, std::placeholders::_2);
        m_mavconn_ptr = std::make_shared<mavconn::MAVConnUDP>(
          driver_system_id, driver_component_id,
          bind_host, bind_port,
          remote_host, remote_port
        );
        m_mavconn_ptr->set_protocol_version(mavconn::Protocol::V10);
        m_mavconn_ptr->message_received_cb = cbk;
        ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
        m_is_connected = true;
      }
      catch (mavconn::DeviceError& e)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Could not connect to sensor: " << e.what());
        m_is_connected = false;
      }

      return m_is_connected;
    }

    //}

    /* heartbeat_loop() //{ */
    void heartbeat_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      send_heartbeat();
    }
    //}

    /* command_loop() //{ */
    void command_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      const auto [cmd_pitch, cmd_roll, cmd_yaw] = mrs_lib::get_mutexed(m_cmd_mtx, m_cmd_pitch, m_cmd_roll, m_cmd_yaw);
      command_mount(cmd_pitch, cmd_roll, cmd_yaw);
    }
    //}

    /* resend_loop() //{ */
    void resend_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      std::scoped_lock lck(m_mavconn_mtx);
      const auto now = ros::Time::now();
      for (auto& el : m_mavconn_awaiting_ack)
      {
        if (now - el.last_try > m_resend_duration)
        {
          m_mavconn_ptr->send_message(el.msg);
          el.last_try = now;
          el.tries++;
        }
      }

      for (const auto& el : m_mavconn_awaiting_ack)
      {
        if (el.tries >= m_resend_max_tries)
          ROS_ERROR_STREAM("[Gimbal]: Giving up on command " << el.msg.command << " after " << el.tries << " attempts (not received ACK). Sorry.");
      }
      m_mavconn_awaiting_ack.erase(
          std::remove_if(
            std::begin(m_mavconn_awaiting_ack), std::end(m_mavconn_awaiting_ack),
            [this](const auto& el)
            {
              return el.tries >= m_resend_max_tries;
            }), std::end(m_mavconn_awaiting_ack));
    }
    //}

    /* send_heartbeat() method //{ */
    void send_heartbeat()
    {
      // Define the system type and some other magic MAVLink constants
      constexpr auto type = mavlink::common::MAV_TYPE::QUADROTOR;               ///< This system is a quadrotor (it's maybe not but it doesn't matter)
      constexpr auto autopilot_type = mavlink::common::MAV_AUTOPILOT::INVALID;  ///< don't even know why this
      constexpr auto system_mode = mavlink::common::MAV_MODE::PREFLIGHT;        ///< Booting up
      constexpr uint32_t custom_mode = 0;                                       ///< Custom mode, can be defined by user/adopter
      constexpr auto system_state = mavlink::common::MAV_STATE::STANDBY;        ///< System ready for flight

      mavlink::common::msg::HEARTBEAT msg;
      msg.type = static_cast<uint8_t>(type);
      msg.autopilot = static_cast<uint8_t>(autopilot_type);
      msg.base_mode = static_cast<uint8_t>(system_mode);
      msg.custom_mode = custom_mode;
      msg.system_status = static_cast<uint8_t>(system_state);
      // send the heartbeat message
      ROS_INFO_STREAM_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending heartbeat.");
      std::scoped_lock lck(m_mavconn_mtx);
      m_mavconn_ptr->send_message(msg);
    }
    //}

    /* send_attitude() method //{ */
    void send_attitude(const float roll, const float pitch, const float yaw, const float rollspeed, const float pitchspeed, const float yawspeed)
    {
      const uint32_t time_boot_ms = (ros::Time::now() - m_start_time).toSec()*1000;
      mavlink::common::msg::ATTITUDE msg;
      msg.time_boot_ms = time_boot_ms;
      msg.roll = roll;
      msg.pitch = pitch;
      msg.yaw = yaw;
      msg.rollspeed = rollspeed;
      msg.pitchspeed = pitchspeed;
      msg.yawspeed = yawspeed;

      // send the heartbeat message
      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending attitude RPY: [%.0f, %.0f, %.0f]deg, RPY vels: [%.0f, %.0f, %.0f]deg/s.", rad2deg(roll), rad2deg(pitch), rad2deg(yaw), rad2deg(rollspeed), rad2deg(pitchspeed), rad2deg(yawspeed));
      std::scoped_lock lck(m_mavconn_mtx);
      m_mavconn_ptr->send_message(msg);
    }
    //}

    /* send_global_position_int() method //{ */
    void send_global_position_int()
    {
      const uint32_t time_boot_ms = (ros::Time::now() - m_start_time).toSec()*1000;
      mavlink::common::msg::GLOBAL_POSITION_INT msg;
      msg.time_boot_ms = time_boot_ms;
      msg.lat = 0;
      msg.lon = 0;
      msg.alt = 0;
      msg.relative_alt = 0;
      msg.vx = 0;
      msg.vy = 0;
      msg.vz = 0;
      msg.hdg = 0;
      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending global position int (all zeros)");
      std::scoped_lock lck(m_mavconn_mtx);
      m_mavconn_ptr->send_message(msg);
    }
    //}

    /* attitude_cbk() method //{ */
    void attitude_cbk(nav_msgs::Odometry::ConstPtr odometry_in)
    {
      const geometry_msgs::Quaternion orientation_quat = odometry_in->pose.pose.orientation;
      const mat3_t rot_mat(quat_t(orientation_quat.w, orientation_quat.x, orientation_quat.y, orientation_quat.z));
      constexpr int ROLL_IDX = 0;
      constexpr int PITCH_IDX = 1;
      constexpr int YAW_IDX = 2;
      // TODO: fix...
      /* const vec3_t PRY_angles = rot_mat.eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX); */
      const vec3_t PRY_angles = rot_mat.eulerAngles(ROLL_IDX, YAW_IDX, PITCH_IDX);
      const float pitch = static_cast<float>(PRY_angles.x());
      const float roll = static_cast<float>(PRY_angles.y());
      const float yaw = static_cast<float>(PRY_angles.z());

      const float pitchspeed = odometry_in->twist.twist.angular.y;
      const float rollspeed = odometry_in->twist.twist.angular.x;
      const float yawspeed = odometry_in->twist.twist.angular.z;

      send_attitude(pitch, roll, yaw, pitchspeed, rollspeed, yawspeed);
      send_global_position_int();
    }
    //}

    /* cmd_orientation_cbk() method //{ */
    void cmd_orientation_cbk(const geometry_msgs::QuaternionStamped::ConstPtr cmd_orientation)
    {
      const auto ori_opt = m_transformer.transformSingle(cmd_orientation, m_base_frame_id);
      if (!ori_opt.has_value())
      {
        ROS_ERROR_THROTTLE(1.0, "[Gimbal]: Could not transform commanded orientation from frame %s to %s, ignoring.", cmd_orientation->header.frame_id.c_str(), m_base_frame_id.c_str());
        return;
      }
      const geometry_msgs::Quaternion orientation_quat = ori_opt.value()->quaternion;
      const mat3_t rot_mat(quat_t(orientation_quat.w, orientation_quat.x, orientation_quat.y, orientation_quat.z));
      constexpr int ROLL_IDX = 0;
      constexpr int PITCH_IDX = 1;
      constexpr int YAW_IDX = 2;
      // TODO: fix...
      /* const vec3_t PRY_angles = rot_mat.eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX); */
      const vec3_t PRY_angles = rot_mat.eulerAngles(ROLL_IDX, YAW_IDX, PITCH_IDX);
      const double pitch = PRY_angles.x();
      const double roll = PRY_angles.y();
      const double yaw = PRY_angles.z();

      mrs_lib::set_mutexed(m_cmd_mtx, std::make_tuple(pitch, roll, yaw), std::forward_as_tuple(m_cmd_pitch, m_cmd_roll, m_cmd_yaw));
    }
    //}

    /* cmd_pry_cbk() method //{ */
    void cmd_pry_cbk(const mrs_msgs::GimbalPRY::ConstPtr cmd_pry)
    {
      const double pitch = static_cast<double>(cmd_pry->pitch);
      const double roll = static_cast<double>(cmd_pry->roll);
      const double yaw = static_cast<double>(cmd_pry->yaw);
      mrs_lib::set_mutexed(m_cmd_mtx, std::make_tuple(pitch, roll, yaw), std::forward_as_tuple(m_cmd_pitch, m_cmd_roll, m_cmd_yaw));
    }
    //}

    /* stream_mode_cbk() method //{ */
    void stream_mode_cbk(const std_msgs::String::ConstPtr stream_mode_msg)
    {
      stream_mode_t stream_mode;
      bool found = false;
      for (int it = 0; it < int(stream_mode_t::side_by_side); it++)
      {
        if (to_str(stream_mode_t(it)) == stream_mode_msg->data)
        {
          stream_mode = stream_mode_t(it);
          found = true;
          break;
        }
      }
      if (found)
        set_stream(stream_mode);
      else
        ROS_ERROR("[Gimbal]: Unknown stream mode: %s", stream_mode_msg->data.c_str());
    }
    //}

    /* command_mount() method //{ */
    void command_mount(const double pitch, const double roll, const double yaw)
    {
      // recalculate to degrees
      const float pitch_deg = static_cast<float>(pitch/M_PI*180.0);
      const float roll_deg = static_cast<float>(roll/M_PI*180.0);
      const float yaw_deg = static_cast<float>(yaw/M_PI*180.0);

      mavlink::common::msg::COMMAND_LONG msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::DO_DIGICAM_CONTROL);
      msg.confirmation = 0;
      msg.param1 = static_cast<float>(OS_Cmd_t::set_system_mode);
      msg.param2 = static_cast<float>(camera_mode_t::global_position);
      msg.param3 = static_cast<float>(pitch_deg);
      msg.param4 = static_cast<float>(roll_deg);
      msg.param5 = static_cast<float>(0);
      msg.param6 = static_cast<float>(0);
      msg.param7 = static_cast<float>(0);

      const quat_t q = pry2quat(pitch, 0.0, roll, false);
      nav_msgs::OdometryPtr ros_msg = boost::make_shared<nav_msgs::Odometry>();
      ros_msg->header.frame_id = m_base_frame_id;
      ros_msg->header.stamp = ros::Time::now();
      ros_msg->child_frame_id = m_child_frame_id;
      ros_msg->pose.pose.orientation.x = q.x();
      ros_msg->pose.pose.orientation.y = q.y();
      ros_msg->pose.pose.orientation.z = q.z();
      ros_msg->pose.pose.orientation.w = q.w();
      m_pub_command.publish(ros_msg);

      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending mount control command (pitch: %.0fdeg, roll: %.0fdeg).", pitch_deg, roll_deg);
      send_with_ack(std::move(msg));
    }
    //}

    /* set_stream() method //{ */
    void set_stream(const stream_mode_t stream_mode, const pip_mode_t pip_mode = pip_mode_t::pip_day_large, const sbs_mode_t sbs_mode = sbs_mode_t::day_left_ir_right)
    {
      mavlink::common::msg::COMMAND_LONG msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::DO_DIGICAM_CONTROL);
      msg.confirmation = 0;
      msg.param1 = static_cast<float>(OS_Cmd_t::stream_control);
      msg.param2 = static_cast<float>(stream_command_t::set_stream_mode);
      msg.param3 = static_cast<float>(stream_mode);
      msg.param4 = static_cast<float>(0);
      msg.param5 = static_cast<float>(0);
      msg.param6 = static_cast<float>(0);
      msg.param7 = static_cast<float>(0);

      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Setting stream mode to %s.", to_str(stream_mode).c_str());
      send_with_ack(std::move(msg));
    }
    //}

    /* check_awaiting_ack() method //{ */
    bool check_awaiting_ack(const mavlink::common::msg::COMMAND_LONG& msg)
    {
      std::scoped_lock lck(m_mavconn_mtx);
      const auto found_it = std::find_if(
          std::begin(m_mavconn_awaiting_ack), std::end(m_mavconn_awaiting_ack),
          [&msg](const auto& el)
          {
            return el.msg.command == msg.command;
          }
        );
      if (found_it != std::end(m_mavconn_awaiting_ack))
      {
        ROS_ERROR_STREAM("[Gimbal]: Command with ID " << msg.command << " already in the queue waiting for ACK from the gimbal, ignoring new message.");
        return true;
      }
      else
        return false;
    }
    //}

    /* send_with_ack() method //{ */
    void send_with_ack(const mavlink::common::msg::COMMAND_LONG& msg)
    {
      std::scoped_lock lck(m_mavconn_mtx);
      if (check_awaiting_ack(msg))
        return;
      m_mavconn_ptr->send_message(msg);
      m_mavconn_awaiting_ack.push_back({std::move(msg), ros::Time::now()});
    }
    //}

    /* msg_callback() method //{ */
    void msg_callback(const mavlink::mavlink_message_t* msg, const mavconn::Framing framing)
    {
      if (framing != mavconn::Framing::ok)
      {
        ROS_ERROR_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Received message with bad framing, ignoring.");
        return;
      }
      mavlink::MsgMap msg_map(msg);

      // Handle message
      switch (msg->msgid)
      {
        case mavlink::common::msg::HEARTBEAT::MSG_ID:  // #0: Heartbeat
        {
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal heartbeat.");
        }
        break;

        case mavlink::common::msg::SYS_STATUS::MSG_ID:  // #1: SYS_STATUS
        {
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal status.");
          mavlink::common::msg::SYS_STATUS decoded;
          decoded.deserialize(msg_map);
        }
        break;

        case mavlink::common::msg::PARAM_VALUE::MSG_ID:  // #22: PARAM_VALUE
        {
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal param value.");
          mavlink::common::msg::PARAM_VALUE decoded;
          decoded.deserialize(msg_map);
        }
        break;

        case mavlink::common::msg::ATTITUDE::MSG_ID:  // #30: ATTITUDE
        {
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal attitude.");
          mavlink::common::msg::ATTITUDE decoded;
          decoded.deserialize(msg_map);
        }
        break;

        case mavlink::common::msg::COMMAND_ACK::MSG_ID:  // #77: COMMAND_ACK
        {
          mavlink::common::msg::COMMAND_ACK decoded;
          decoded.deserialize(msg_map);
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal ack of command #%u: %u.", decoded.command, decoded.result);
          m_mavconn_awaiting_ack.erase(std::remove_if(
              std::begin(m_mavconn_awaiting_ack), std::end(m_mavconn_awaiting_ack),
              [&decoded](const auto& el)
              {
                return el.msg.command == decoded.command;
              }
            ), std::end(m_mavconn_awaiting_ack));
        }
        break;

        case mavlink::common::msg::V2_EXTENSION::MSG_ID:  // #248: V2_EXTENSION
        {
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving V2 extension from sysid %d, compid %d.", msg->sysid, msg->compid);
          process_extension_msg(msg);
        }
        break;

        default:
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving unhandled message #%u, ignoring.", msg->msgid);
          break;
      }
    }
    //}

    // fujky
    template<typename T>
    T parse(const uint8_t* buffer, uint8_t& offset)
    {
      const uint16_t ret = *(static_cast<const T*>(static_cast<const void*>(buffer+offset)));
      offset += sizeof(T);
      return ret;
    }

    void parse_system_report(const mavlink::mavlink_message_t* msg)
    {
      const uint8_t* payload = static_cast<const uint8_t*>(static_cast<const void*>(msg->payload64));
      constexpr uint8_t i0 = 6;
      uint8_t i = 8-i0;
      const auto roll_deg = parse<float>(payload, i);
      const auto pitch_deg = parse<float>(payload, i);
      const auto fov = parse<float>(payload, i);

      i = 24-i0;
      const auto system_state = static_cast<system_state_t>(parse<uint8_t>(payload, i));

      // skip some uninteresting stuff
      i = 51-i0;
      const auto roll_rate_degs = parse<float>(payload, i);
      const auto pitch_rate_degs = parse<float>(payload, i);
      const auto camera_temp = parse<float>(payload, i);

      const double roll = roll_deg/180.0*M_PI;
      const double pitch = pitch_deg/180.0*M_PI;
      const double roll_rate = roll_rate_degs/180.0*M_PI;
      const double pitch_rate = pitch_rate_degs/180.0*M_PI;

      const quat_t q = pry2quat(pitch, 0.0, roll, false);
      nav_msgs::OdometryPtr ros_msg = boost::make_shared<nav_msgs::Odometry>();
      ros_msg->header.frame_id = m_base_frame_id;
      ros_msg->header.stamp = ros::Time::now();
      ros_msg->child_frame_id = m_child_frame_id;
      ros_msg->pose.pose.orientation.x = q.x();
      ros_msg->pose.pose.orientation.y = q.y();
      ros_msg->pose.pose.orientation.z = q.z();
      ros_msg->pose.pose.orientation.w = q.w();
      ros_msg->twist.twist.angular.x = roll_rate;
      ros_msg->twist.twist.angular.y = pitch_rate;
      m_pub_attitude.publish(ros_msg);

      ROS_INFO_THROTTLE(1.0, "[Gimbal]: System state: %s\n\troll: %.2fdeg (%.2fdeg/s)\n\tpitch: %.2fdeg (%.2fdeg/s)\n\tfov: %.2fdeg\n\tcamera temp: %.2fdeg C", to_str(system_state).c_str(), roll_deg, roll_rate_degs, pitch_deg, pitch_rate_degs, fov, camera_temp);
    }

    void process_extension_msg(const mavlink::mavlink_message_t* msg)
    {
      const uint8_t* payload = static_cast<const uint8_t*>(static_cast<const void*>(msg->payload64));
      constexpr uint8_t i0 = 6;
      uint8_t i = 6-i0;
      const report_type_t report_type = static_cast<report_type_t>(parse<uint16_t>(payload, i));

      switch (report_type)
      {
        case report_type_t::system: parse_system_report(msg); break;
        default: ROS_WARN_THROTTLE(1.0, "[Gimbal]: Received extension message with %s report type, ignoring.", to_str(report_type).c_str());
      }
    }

    quat_t rpy2quat(const double roll, const double pitch, const double yaw, const bool extrinsic)
    {
      if (extrinsic)
        return ( anax_t(roll, vec3_t::UnitX()) * anax_t(pitch, vec3_t::UnitY()) * anax_t(yaw, vec3_t::UnitZ()));
      else
        return ( anax_t(yaw, vec3_t::UnitZ()) * anax_t(pitch, vec3_t::UnitY()) * anax_t(roll, vec3_t::UnitX()));
    }

    quat_t pry2quat(const double pitch, const double roll, const double yaw, const bool extrinsic)
    {
      if (extrinsic)
        return ( anax_t(-pitch, vec3_t::UnitY()) * anax_t(roll, -vec3_t::UnitX()) * anax_t(yaw, vec3_t::UnitZ()) );
      else
        return ( anax_t(yaw, vec3_t::UnitZ()) * anax_t(roll, -vec3_t::UnitX()) * anax_t(-pitch, vec3_t::UnitY()) );
    }

    // --------------------------------------------------------------
    // |                    ROS-related variables                   |
    // --------------------------------------------------------------
    ros::NodeHandle m_nh;
    ros::Timer m_tim_heartbeat;
    ros::Timer m_tim_command;
    ros::Timer m_tim_receiving;

    ros::Subscriber m_sub_attitude;
    ros::Subscriber m_sub_command;
    ros::Subscriber m_sub_pry;
    ros::Subscriber m_sub_stream_mode;

    ros::Publisher m_pub_attitude;
    ros::Publisher m_pub_command;

    tf2_ros::TransformBroadcaster m_pub_transform;

    mrs_lib::Transformer m_transformer;

    // --------------------------------------------------------------
    // |                 Parameters, loaded from ROS                |
    // --------------------------------------------------------------
    std::string m_uav_name;
    std::string m_portname;
    int m_baudrate;
    ros::Duration m_resend_duration;
    int m_resend_max_tries;

    int m_driver_system_id;
    int m_driver_component_id;

    int m_gimbal_system_id;
    int m_gimbal_component_id;

    std::vector<int> m_stream_request_ids;
    std::vector<int> m_stream_request_rates;

    std::string m_base_frame_id;
    std::string m_child_frame_id;

    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------
    const ros::Time m_start_time = ros::Time::now();
    bool m_is_connected = false;
    std::recursive_mutex m_mavconn_mtx;
    std::vector<awaiting_ack_t> m_mavconn_awaiting_ack;
    mavconn::MAVConnUDP::Ptr m_mavconn_ptr = nullptr;

    std::mutex m_cmd_mtx;
    double m_cmd_pitch = 0.0;
    double m_cmd_roll = 0.0;
    double m_cmd_yaw = 0.0;

    int m_hbs_since_last_request = 2;
    int m_hbs_request_period = 5;

    int m_hbs_since_last_configure = 3;
    int m_hbs_configure_period = 5;

    int m_hbs_since_last_param_request = 4;
    int m_hbs_param_request_period = 5;

    size_t m_chars_received = 0;
    size_t m_valid_chars_received = 0;
  };

  //}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet);
