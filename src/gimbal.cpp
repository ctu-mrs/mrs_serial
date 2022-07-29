/* some code copied from MAVLInk_DroneLights by Juan Pedro López */

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
#include <mavconn/interface.h>

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

#include <mavlink/v2.0/common/mavlink_msg_command_long.hpp>
#include <mavlink/v2.0/common/mavlink_msg_gimbal_device_set_attitude.hpp>
#include <mavlink/v2.0/common/mavlink_msg_gimbal_manager_set_attitude.hpp>

namespace gimbal
{

  /* class Gimbal //{ */

  class Gimbal : public nodelet::Nodelet
  {
  private:
    /* enums and struct defines //{ */

    static constexpr std::array<char, 16> EULER_ORDER_PARAM_ID = {"G_EULER_ORDER\0"};

    enum class euler_order_t
    {
      pitch_roll_yaw = 0,
      roll_pitch_yaw = 1,
      pitchmotor_roll_yawmotor = 2,
      roll_pitchmotor_yawmotor = 3,
      yaw_roll_pitch = 4,
      unknown,
    };

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

    // https://mavlink.io/en/messages/common.html

    enum MAV_CMD
    {
      MAV_CMD_DO_MOUNT_CONFIGURE = 204,  //< Mission command to configure a camera or antenna mount
      MAV_CMD_DO_MOUNT_CONTROL = 205,    //< Mission command to control a camera or antenna mount
    };

    enum MAV_MOUNT_MODE
    {
      MAV_MOUNT_MODE_RETRACT = 0,            //<	Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
      MAV_MOUNT_MODE_NEUTRAL = 1,            //<	Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
      MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,  //<	Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
      MAV_MOUNT_MODE_RC_TARGETING = 3,       //<	Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
      MAV_MOUNT_MODE_GPS_POINT = 4,          //<	Load neutral position and start to point to Lat,Lon,Alt
      MAV_MOUNT_MODE_SYSID_TARGET = 5,       //<	Gimbal tracks system with specified system ID
      MAV_MOUNT_MODE_HOME_LOCATION = 6,      //<	Gimbal tracks home location
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
      const auto url = pl.loadParam2<std::string>("url");
      pl.loadParam("base_frame_id", m_base_frame_id);
      pl.loadParam("child_frame_id", m_child_frame_id);

      pl.loadParam("mavlink/heartbeat_period", m_heartbeat_period, ros::Duration(1.0));
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
      ROS_INFO_THROTTLE(1.0, "[%s] url: %s", ros::this_node::getName().c_str(), url.c_str());

      const bool connected = connect(url);
      if (connected)
      {
        m_tim_sending = m_nh.createTimer(m_heartbeat_period, &Gimbal::sending_loop, this);
        m_mavconn_ptr->message_received_cb = std::bind(&Gimbal::msg_callback, this, std::placeholders::_1, std::placeholders::_2);

        m_pub_attitude = m_nh.advertise<nav_msgs::Odometry>("attitude_out", 10);
        m_pub_command = m_nh.advertise<nav_msgs::Odometry>("current_setpoint", 10);

        m_sub_attitude = m_nh.subscribe("attitude_in", 10, &Gimbal::attitude_cbk, this);
        m_sub_command = m_nh.subscribe("cmd_orientation", 10, &Gimbal::cmd_orientation_cbk, this);
        m_sub_pry = m_nh.subscribe("cmd_pry", 10, &Gimbal::cmd_pry_cbk, this);

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

    bool connect(const std::string& url)
    {
      ROS_INFO_THROTTLE(1.0, "[%s]: Openning the UDP connection.", ros::this_node::getName().c_str());

      try
      {
        m_mavconn_ptr = mavconn::MAVConnInterface::open_url(url);
        m_mavconn_ptr->set_system_id(m_driver_system_id);
        m_mavconn_ptr->set_component_id(m_driver_component_id);
        m_mavconn_ptr->set_protocol_version(mavconn::Protocol::V20);
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

    /* sending_loop() //{ */
    void sending_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      send_heartbeat();

      // request data from the gimbal every N heartbeats
      // to make sure the gimbal hears it and not to
      // overwhelm the serial line at the same time
      m_hbs_since_last_request++;
      if (m_hbs_since_last_request >= m_hbs_request_period)
      {
        request_data(m_stream_request_ids, m_stream_request_rates);
        m_hbs_since_last_request = 0;
      }

      // configure the gimbal every N heartbeats
      // to make sure the gimbal hears it and not to
      // overwhelm the serial line at the same time
      m_hbs_since_last_configure++;
      if (m_hbs_since_last_configure >= m_hbs_configure_period)
      {
        configure_mount(m_mount_config);
        m_hbs_since_last_configure = 0;
      }

      // request the euler order every N heartbeats
      // to make sure the gimbal hears it and not to
      // overwhelm the serial line at the same time
      m_hbs_since_last_param_request++;
      if (m_hbs_since_last_param_request >= m_hbs_param_request_period)
      {
        request_parameter_list();
        m_hbs_since_last_param_request = 0;
      }

      const uint32_t desired_euler_order = static_cast<uint32_t>(euler_order_t::pitchmotor_roll_yawmotor);
      set_parameter(EULER_ORDER_PARAM_ID, *((float*)(&desired_euler_order)), 5);
    }
    //}

    /* request_data() method //{ */
    bool request_data(const std::vector<int>& msg_request_ids, const std::vector<int>& msg_request_intervals)
    {
      bool success = true;

      for (int it = 0; it < msg_request_ids.size(); it++)
      {
        const int request_msg_id_loaded = msg_request_ids.at(it);
        if (request_msg_id_loaded > std::numeric_limits<uint16_t>::max() || request_msg_id_loaded < 0)
        {
          ROS_ERROR("[Gimbal]: Cannot request message ID larger than %u and smaller thatn 0 (got %d)!", std::numeric_limits<uint16_t>::max(), request_msg_id_loaded);
          success = false;
          continue;
        }
        const uint16_t request_msg_id = static_cast<uint16_t>(request_msg_id_loaded);

        const int request_msg_interval_loaded = msg_request_intervals.at(it);
        if (request_msg_interval_loaded > std::numeric_limits<int32_t>::max())
        {
          ROS_ERROR("[Gimbal]: Cannot request message interval larger than %u (got %d)!", std::numeric_limits<int32_t>::max(), request_msg_interval_loaded);
          success = false;
          continue;
        }
        const int32_t request_msg_interval = static_cast<int32_t>(request_msg_interval_loaded);

        const bool start = true;
        ROS_INFO("[Gimbal]: |Driver > Gimbal| Requesting message #%u at interval %uus", request_msg_id, request_msg_interval);
        mavlink::common::msg::MESSAGE_INTERVAL msg;
        msg.message_id = request_msg_id;
        msg.interval_us = request_msg_interval;
        m_mavconn_ptr->send_message(msg);
      }

      // TODO: Request the value of the "EULER_ORDER" parameter to know in what format does the attitude arrive
      /* mavlink_msg_param_value_get_param_id() */
      return success;
    }
    //}

    /* request_parameter_list() method //{ */
    void request_parameter_list()
    {
      // Request the list of all parameters
      ROS_INFO("[Gimbal]: |Driver > Gimbal| Requesting parameter list");
      mavlink::common::msg::PARAM_REQUEST_LIST msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      m_mavconn_ptr->send_message_ignore_drop(msg);
    }
    //}

    /* request_parameter() method //{ */
    void request_parameter(const std::array<char, 16>& param_id)
    {
      // Request the value of the "EULER_ORDER" parameter to know in what format does the attitude arrive
      std::stringstream id_str;
      for (const auto& c : param_id)
        id_str << c;
      ROS_INFO_STREAM("[Gimbal]: |Driver > Gimbal| Requesting parameter ID " << id_str.str());
      mavlink::common::msg::PARAM_REQUEST_READ msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.param_id = param_id;
      msg.param_index = -1; // -1 means use the param_id instead of index
      m_mavconn_ptr->send_message(msg);
    }
    //}

    /* set_parameter() method //{ */
    void set_parameter(const std::array<char, 16>& param_id, const float param_value, const uint8_t param_type)
    {
      // Set the value of the specified parameter to the specified value
      std::stringstream id_str;
      for (const auto& c : param_id)
        id_str << c;
      ROS_INFO("[Gimbal]: |Driver > Gimbal| Setting parameter id %s to %f (type %u)", id_str.str().c_str(), param_value, param_type);
      mavlink::common::msg::PARAM_SET msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.param_id = param_id;
      msg.param_value = param_value;
      msg.param_type = param_type;
      m_mavconn_ptr->send_message(msg);
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
      m_mavconn_ptr->send_message(msg);
    }
    //}

    /* configure_mount() method //{ */
    void configure_mount(const mount_config_t& mount_config)
    {
      mavlink::common::msg::COMMAND_LONG msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::DO_MOUNT_CONFIGURE);
      msg.confirmation = 0;
      msg.param1 = static_cast<float>(mavlink::common::MAV_MOUNT_MODE::MAVLINK_TARGETING);
      msg.param2 = static_cast<float>(mount_config.stabilize_roll);
      msg.param3 = static_cast<float>(mount_config.stabilize_pitch);
      msg.param4 = static_cast<float>(mount_config.stabilize_yaw);
      msg.param5 = static_cast<float>(mount_config.roll_input_mode);
      msg.param6 = static_cast<float>(mount_config.pitch_input_mode);
      msg.param7 = static_cast<float>(mount_config.yaw_input_mode);
      ROS_INFO_THROTTLE(1.0,
                        "[Gimbal]: |Driver > Gimbal| Sending mount configuration command (stab. roll: %d, stab. pitch: %d, stab. yaw: %d, roll mode: %d, pitch "
                        "mode: %d, yaw mode: %d).",
                        mount_config.stabilize_roll, mount_config.stabilize_pitch, mount_config.stabilize_yaw, (int)mount_config.roll_input_mode,
                        (int)mount_config.pitch_input_mode, (int)mount_config.yaw_input_mode);
      m_mavconn_ptr->send_message_ignore_drop(msg);
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
    void cmd_orientation_cbk(geometry_msgs::QuaternionStamped::ConstPtr cmd_orientation)
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
      command_mount(pitch, roll, yaw);
    }
    //}

    /* cmd_pry_cbk() method //{ */
    void cmd_pry_cbk(mrs_msgs::GimbalPRY::ConstPtr cmd_pry)
    {
      command_mount(cmd_pry->pitch, cmd_pry->roll, cmd_pry->yaw);
    }
    //}

    /* command_mount() method //{ */
    void command_mount(const double pitch, const double roll, const double yaw)
    {
      // recalculate to degrees
      const float pitch_deg = static_cast<float>(pitch/M_PI*180.0);
      const float roll_deg = static_cast<float>(roll/M_PI*180.0);
      const float yaw_deg = static_cast<float>(yaw/M_PI*180.0);

      // MAV_CMD_DO_MOUNT_CONTROL command parameters (https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL):
      /* 1: Pitch	      pitch depending on mount mode (degrees or degrees/second depending on pitch input). */
      /* 2: Roll	      roll depending on mount mode (degrees or degrees/second depending on roll input). */
      /* 3: Yaw	        yaw depending on mount mode (degrees or degrees/second depending on yaw input). */
      /* 4: Altitude	  altitude depending on mount mode.	(m) */
      /* 5: Latitude	  latitude, set if appropriate mount mode. */
      /* 6: Longitude	  longitude, set if appropriate mount mode. */
      /* 7: Mode	      Mount mode.	(MAV_MOUNT_MODE) */

      mavlink::common::msg::COMMAND_LONG msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::DO_MOUNT_CONTROL);
      msg.confirmation = 0;
      msg.param1 = static_cast<float>(pitch_deg);
      msg.param2 = static_cast<float>(roll_deg);
      msg.param3 = static_cast<float>(yaw_deg);
      msg.param4 = static_cast<float>(0);
      msg.param5 = static_cast<float>(0);
      msg.param6 = static_cast<float>(0);
      msg.param7 = static_cast<float>(mavlink::common::MAV_MOUNT_MODE::MAVLINK_TARGETING);
      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending mount control command (pitch: %.0fdeg, roll: %.0fdeg, yaw: %.0fdeg).", pitch_deg, roll_deg, yaw_deg);
      m_mavconn_ptr->send_message(msg);

      const quat_t q = pry2quat(pitch, roll, yaw, false);
      nav_msgs::OdometryPtr ros_msg = boost::make_shared<nav_msgs::Odometry>();
      ros_msg->header.frame_id = m_base_frame_id;
      ros_msg->header.stamp = ros::Time::now();
      ros_msg->child_frame_id = m_child_frame_id;
      ros_msg->pose.pose.orientation.x = q.x();
      ros_msg->pose.pose.orientation.y = q.y();
      ros_msg->pose.pose.orientation.z = q.z();
      ros_msg->pose.pose.orientation.w = q.w();
      m_pub_command.publish(ros_msg);
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
          process_param_value_msg(decoded);
        }
        break;

        case mavlink::common::msg::ATTITUDE::MSG_ID:  // #30
        {
          /* Message decoding: PRIMITIVE
           *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
           */
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal attitude.");
          mavlink::common::msg::ATTITUDE decoded;
          decoded.deserialize(msg_map);
          process_attitude_msg(decoded, m_euler_ordering);
        }
        break;

        default:
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving unhandled message #%u, ignoring.", msg->msgid);
          break;
      }
    }
    //}

    /* process_param_value_msg() method //{ */
    void process_param_value_msg(const mavlink::common::msg::PARAM_VALUE& param_value)
    {
      char param_id[17];
      param_id[16] = 0;
      std::copy_n(std::begin(param_value.param_id), 16, param_id);
    
      if (std::strcmp(param_id, EULER_ORDER_PARAM_ID.data()) == 0)
      {
        // | ----------------------- EULER_ORDER ---------------------- |
        const uint32_t value = *((uint32_t*)(&param_value.param_value));
        if (value >= 0 && value < static_cast<uint32_t>(euler_order_t::unknown))
        {
          m_euler_ordering = static_cast<euler_order_t>(value);
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: %s parameter is %d (raw is %f, type is %u).", EULER_ORDER_PARAM_ID.data(), (int)m_euler_ordering, param_value.param_value, param_value.param_type);
        }
        else
        {
          ROS_ERROR("[Gimbal]: Unknown value of %s received: %f (type: %u), ignoring.", EULER_ORDER_PARAM_ID.data(), param_value.param_value, param_value.param_type);
          m_euler_ordering = euler_order_t::unknown;
        }
      }
      else
      {
        ROS_DEBUG("[Gimbal]: Unhandled parameter value %s received with value %f (type: %u), ignoring.", param_id, param_value.param_value, param_value.param_type);
      }
    }
    //}

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

    /* process_attitude_msg() method //{ */
    void process_attitude_msg(const mavlink::common::msg::ATTITUDE& attitude, const euler_order_t& euler_order)
    {
      // TODO: this is probably wrong, but works so far for "pitchmotor_roll_yawmotor"
      // TODO: complete this for other euler angle orderings if necessary
      quat_t q;
      switch (euler_order)
      {
        case euler_order_t::roll_pitch_yaw:
          q = rpy2quat(attitude.roll, attitude.pitch, attitude.yaw, true);
          break;
        case euler_order_t::pitchmotor_roll_yawmotor:
          q = pry2quat(attitude.pitch, attitude.roll, attitude.yaw, false);
          break;
        case euler_order_t::roll_pitchmotor_yawmotor:
          q = rpy2quat(attitude.roll, attitude.pitch, attitude.yaw, false);
          break;
        case euler_order_t::pitch_roll_yaw:
          q = pry2quat(attitude.pitch, attitude.roll, attitude.yaw, true);
          break;
        case euler_order_t::unknown:
        default:
          ROS_ERROR_THROTTLE(1.0, "[Gimbal]: Unknown euler angle ordering, cannot process attitude.");
          return;
      }

      nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
      msg->header.frame_id = m_base_frame_id;
      msg->header.stamp = ros::Time::now();
      msg->child_frame_id = m_child_frame_id;
      msg->pose.pose.orientation.x = q.x();
      msg->pose.pose.orientation.y = q.y();
      msg->pose.pose.orientation.z = q.z();
      msg->pose.pose.orientation.w = q.w();
      m_pub_attitude.publish(msg);

      geometry_msgs::TransformStamped tf;
      tf.header = msg->header;
      tf.child_frame_id = msg->child_frame_id;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      m_pub_transform.sendTransform(tf);
    }
    //}

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

    std::string m_base_frame_id;
    std::string m_child_frame_id;

    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------
    const ros::Time m_start_time = ros::Time::now();
    ros::Time m_last_sent_time = ros::Time::now();
    bool m_is_connected = false;
    mavconn::MAVConnInterface::Ptr m_mavconn_ptr = nullptr;

    int m_hbs_since_last_request = 2;
    int m_hbs_request_period = 5;

    int m_hbs_since_last_configure = 3;
    int m_hbs_configure_period = 5;

    int m_hbs_since_last_param_request = 4;
    int m_hbs_param_request_period = 5;

    size_t m_chars_received = 0;
    size_t m_valid_chars_received = 0;

    euler_order_t m_euler_ordering = euler_order_t::unknown;
  };

  //}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet);
