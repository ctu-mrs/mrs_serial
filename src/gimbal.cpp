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
#include "mavlink/mavlink.h"
#include "serial_port.h"

namespace gimbal
{

  /* class Gimbal //{ */

  class Gimbal : public nodelet::Nodelet
  {
  private:
    /* enums and struct defines //{ */

    static constexpr char EULER_ORDER_PARAM_ID[16] = "G_EULER_ORDER\0";

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

  public:
    /* onInit() //{ */

    virtual void onInit() override
    {

      // Get paramters
      m_nh = ros::NodeHandle("~");

      ros::Time::waitForValid();

      mrs_lib::ParamLoader pl(m_nh);

      pl.loadParam("uav_name", m_uav_name);
      pl.loadParam("portname", m_portname);
      pl.loadParam("baudrate", m_baudrate);
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
      ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), m_portname.c_str());
      ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), m_baudrate);

      const bool connected = connect();
      if (connected)
      {
        m_tim_sending = m_nh.createTimer(m_heartbeat_period, &Gimbal::sending_loop, this);
        m_tim_receiving = m_nh.createTimer(ros::Duration(0.05), &Gimbal::receiving_loop, this);
        m_pub_attitude = m_nh.advertise<nav_msgs::Odometry>("gimbal_attitude", 10);
        m_pub_command = m_nh.advertise<nav_msgs::Odometry>("gimbal_command_dbg", 10);
        m_sub_command = m_nh.subscribe("cmd_orientation", 10, &Gimbal::cmd_orientation_cbk, this);
        m_sub_pry = m_nh.subscribe("cmd_pry", 10, &Gimbal::cmd_pry_cbk, this);
        m_transformer = mrs_lib::Transformer("Gimbal", m_uav_name);
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

    bool connect(void)
    {
      ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

      if (!m_serial_port.connect(m_portname, m_baudrate))
      {
        ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
        m_is_connected = false;
      } else
      {
        ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
        m_is_connected = true;
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
        const bool success = request_data(m_stream_request_ids, m_stream_request_rates);
        if (success)
          m_hbs_since_last_request = 0;
      }

      // configure the gimbal every N heartbeats
      // to make sure the gimbal hears it and not to
      // overwhelm the serial line at the same time
      m_hbs_since_last_configure++;
      if (m_hbs_since_last_configure >= m_hbs_configure_period)
      {
        const bool success = configure_mount(m_mount_config);
        if (success)
          m_hbs_since_last_configure = 0;
      }

      // request the euler order every N heartbeats
      // to make sure the gimbal hears it and not to
      // overwhelm the serial line at the same time
      m_hbs_since_last_param_request++;
      if (m_hbs_since_last_param_request >= m_hbs_param_request_period)
      {
        const bool success = request_parameter_list();
        /* const bool success = request_parameter(EULER_ORDER_PARAM_ID); */
        if (success)
          m_hbs_since_last_param_request = 0;
      }

      const uint32_t desired_euler_order = static_cast<uint32_t>(euler_order_t::pitchmotor_roll_yawmotor);
      set_parameter(EULER_ORDER_PARAM_ID, *((float*)(&desired_euler_order)), 5);
    }
    //}

    /* request_data() method //{ */
    bool request_data(const std::vector<int>& stream_request_ids, const std::vector<int>& stream_request_rates)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      // STREAMS that can be requested
      /*
       * Definitions are in common.h: enum MAV_DATA_STREAM
       *
       * MAV_DATA_STREAM_ALL=0,             * Enable all data streams
       * MAV_DATA_STREAM_RAW_SENSORS=1,     * Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
       * MAV_DATA_STREAM_EXTENDED_STATUS=2, * Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
       * MAV_DATA_STREAM_RC_CHANNELS=3,     * Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
       * MAV_DATA_STREAM_RAW_CONTROLLER=4,  * Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
       * MAV_DATA_STREAM_POSITION=6,        * Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
       * MAV_DATA_STREAM_EXTRA1=10,         * Dependent on the autopilot
       * MAV_DATA_STREAM_EXTRA2=11,         * Dependent on the autopilot
       * MAV_DATA_STREAM_EXTRA3=12,         * Dependent on the autopilot
       * MAV_DATA_STREAM_ENUM_END=13,
       *
       * Available data from the gimbal is:
       * MAV_DATA_STREAM_RAW_SENSORS      (1)  - RAW_IMU
       * MAV_DATA_STREAM_RC_CHANNELS      (3)  - RC_CHANNELS_SCALED
       * MAV_DATA_STREAM_RAW_CONTROLLER   (4)  - ATTITUDE
       * MAV_DATA_STREAM_EXTRA1           (10) - ATTITUDE
       * MAV_DATA_STREAM_EXTRA2           (11) - ATTITUDE
       */

      // To be setup according to the needed information to be requested from the Pixhawk
      /* const std::vector<std::tuple<uint8_t, uint16_t>> requests = { {MAV_DATA_STREAM_EXTRA1, 200} }; */
      bool success = true;

      for (int it = 0; it < stream_request_ids.size(); it++)
      {
        /*
         * mavlink_msg_request_data_stream_pack(system_id, component_id,
         *    &msg,
         *    target_system, target_component,
         *    MAV_DATA_STREAM_POSITION, 10000000, 1);
         *
         * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
         *    mavlink_message_t* msg,
         *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
         *    uint16_t req_message_rate, uint8_t start_stop)
         *
         */
        const int request_stream_id_loaded = stream_request_ids.at(it);
        if (request_stream_id_loaded > std::numeric_limits<uint8_t>::max())
        {
          ROS_ERROR("[Gimbal]: Cannot request stream ID larger than %u (got %d)!", std::numeric_limits<uint8_t>::max(), request_stream_id_loaded);
          success = false;
          continue;
        }
        const uint8_t request_stream_id = static_cast<uint8_t>(request_stream_id_loaded);

        const int request_stream_rate_loaded = stream_request_rates.at(it);
        if (request_stream_rate_loaded > std::numeric_limits<uint16_t>::max())
        {
          ROS_ERROR("[Gimbal]: Cannot request stream rate larger than %u (got %d)!", std::numeric_limits<uint16_t>::max(), request_stream_rate_loaded);
          success = false;
          continue;
        }
        const uint16_t request_stream_rate = static_cast<uint16_t>(request_stream_rate_loaded);

        const bool start = true;
        ROS_INFO("[Gimbal]: |Driver > Gimbal| Requesting message stream #%u at rate %uHz", request_stream_id, request_stream_rate);
        mavlink_msg_request_data_stream_pack(m_driver_system_id, m_driver_component_id, &msg, m_gimbal_system_id, m_gimbal_component_id, request_stream_id,
                                             request_stream_rate, start);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        success = success && m_serial_port.sendCharArray(buf, len);
      }

      // TODO: Request the value of the "EULER_ORDER" parameter to know in what format does the attitude arrive
      /* mavlink_msg_param_value_get_param_id() */
      return success;
    }
    //}

    /* request_parameter_list() method //{ */
    bool request_parameter_list()
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      // Request the list of all parameters
      /* uint16_t mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index) */
      ROS_INFO("[Gimbal]: |Driver > Gimbal| Requesting parameter list");
      mavlink_msg_param_request_list_pack(m_driver_system_id, m_driver_component_id, &msg, m_gimbal_system_id, m_gimbal_component_id);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      const bool success = m_serial_port.sendCharArray(buf, len);

      return success;
    }
    //}

    /* request_parameter() method //{ */
    bool request_parameter(const char param_id[16])
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      // Request the value of the "EULER_ORDER" parameter to know in what format does the attitude arrive
      /* uint16_t mavlink_msg_param_request_read_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index) */
      ROS_INFO("[Gimbal]: |Driver > Gimbal| Requesting parameter id %s", param_id);
      const int16_t param_index = -1; // -1 means use the param_id instead of index
      mavlink_msg_param_request_read_pack(m_driver_system_id, m_driver_component_id, &msg, m_gimbal_system_id, m_gimbal_component_id, param_id, param_index);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      const bool success = m_serial_port.sendCharArray(buf, len);

      return success;
    }
    //}

    /* set_parameter() method //{ */
    bool set_parameter(const char param_id[16], const float param_value, const uint8_t param_type)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      // Set the value of the "EULER_ORDER" parameter to dictate in what format should the attitude arrive
      /* uint16_t mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type) */
      ROS_INFO("[Gimbal]: |Driver > Gimbal| Setting parameter id %s to %f (type %u)", param_id, param_value, param_type);
      mavlink_msg_param_set_pack(m_driver_system_id, m_driver_component_id, &msg, m_gimbal_system_id, m_gimbal_component_id, param_id, param_value, param_type);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      const bool success = m_serial_port.sendCharArray(buf, len);

      return success;
    }
    //}

    /* send_heartbeat() method //{ */
    bool send_heartbeat()
    {
      // Define the system type and some other magic MAVLink constants
      constexpr uint8_t type = MAV_TYPE_QUADROTOR;               ///< This system is a quadrotor (it's maybe not but it doesn't matter)
      constexpr uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;  ///< don't even know why this
      constexpr uint8_t system_mode = MAV_MODE_PREFLIGHT;        ///< Booting up
      constexpr uint32_t custom_mode = 0;                        ///< Custom mode, can be defined by user/adopter
      constexpr uint8_t system_state = MAV_STATE_STANDBY;        ///< System ready for flight

      // Initialize the required buffers
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      // Pack the message
      mavlink_msg_heartbeat_pack(m_driver_system_id, m_driver_component_id, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

      // Copy the message to the send buffer
      const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      // send the heartbeat message
      ROS_INFO_STREAM_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending heartbeat.");
      return m_serial_port.sendCharArray(buf, len);
    }
    //}

    /* configure_mount() method //{ */
    bool configure_mount(const mount_config_t& mount_config)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      /* uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2,
       * float param3, float param4, float param5, float param6, float param7) */

      // MAV_CMD_DO_MOUNT_CONFIGURE command parameters (https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONFIGURE):
      /* 1: Mode	Mount operation mode (MAV_MOUNT_MODE) */
      /* 2: Stabilize Roll	stabilize roll? (1 = yes, 0 = no)	min:0 max:1 increment:1 */
      /* 3: Stabilize Pitch	stabilize pitch? (1 = yes, 0 = no)	min:0 max:1 increment:1 */
      /* 4: Stabilize Yaw	stabilize yaw? (1 = yes, 0 = no)	min:0 max:1 increment:1 */
      /* 5: Roll Input Mode	roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame) */
      /* 6: Pitch Input Mode	pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame) */
      /* 7: Yaw Input Mode	yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame) */
      mavlink_msg_command_long_pack(m_driver_system_id, m_driver_component_id, &msg,
                                    // tgt. system id,  tgt. component id,      command id,                 confirmation
                                    m_gimbal_system_id, m_gimbal_component_id, MAV_CMD_DO_MOUNT_CONFIGURE, 0,
                                    // command parameters
                                    MAV_MOUNT_MODE_MAVLINK_TARGETING, (float)mount_config.stabilize_roll, (float)mount_config.stabilize_pitch,
                                    (float)mount_config.stabilize_yaw, (float)mount_config.roll_input_mode, (float)mount_config.pitch_input_mode,
                                    (float)mount_config.yaw_input_mode);
      const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      // send the heartbeat message
      ROS_INFO_THROTTLE(1.0,
                        "[Gimbal]: |Driver > Gimbal| Sending mount configuration command (stab. roll: %d, stab. pitch: %d, stab. yaw: %d, roll mode: %d, pitch "
                        "mode: %d, yaw mode: %d).",
                        mount_config.stabilize_roll, mount_config.stabilize_pitch, mount_config.stabilize_yaw, (int)mount_config.roll_input_mode,
                        (int)mount_config.pitch_input_mode, (int)mount_config.yaw_input_mode);
      return m_serial_port.sendCharArray(buf, len);
    }
    //}

    /* cmd_orientation_cbk() method //{ */
    void cmd_orientation_cbk(geometry_msgs::QuaternionStamped::ConstPtr cmd_orientation)
    {
      const auto ori_opt = m_transformer.transformSingle(m_base_frame_id, cmd_orientation);
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
    bool command_mount(const double pitch, const double roll, const double yaw)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      /* uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2,
       * float param3, float param4, float param5, float param6, float param7) */

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
      mavlink_msg_command_long_pack(m_driver_system_id, m_driver_component_id, &msg,
                                    // tgt. system id,  tgt. component id,      command id,               confirmation
                                    m_gimbal_system_id, m_gimbal_component_id, MAV_CMD_DO_MOUNT_CONTROL, 0,
                                    // command parameters
                                    pitch_deg, roll_deg, yaw_deg, 0, 0, 0, MAV_MOUNT_MODE_MAVLINK_TARGETING);
      const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      // send the heartbeat message
      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending mount control command (pitch: %.0fdeg, roll: %.0fdeg, yaw: %.0fdeg).", pitch_deg, roll_deg, yaw_deg);
      const bool ret = m_serial_port.sendCharArray(buf, len);

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

      return ret;
    }
    //}

    /* receiving_loop() method //{ */
    void receiving_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      mavlink_message_t msg;
      mavlink_status_t status;

      uint8_t c;
      while (m_serial_port.readChar(&c))
      {
        m_chars_received++;
        // Try to get a new message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
          // Handle message
          switch (msg.msgid)
          {
            case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
            {
              ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal heartbeat.");
            }
            break;

            case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
            {
              ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal status.");
              /* Message decoding: PRIMITIVE
               *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
               */
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);
            }
            break;

            case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
            {
              ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal param value.");
              /* Message decoding: PRIMITIVE
               *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
               */
              mavlink_param_value_t param_value;
              mavlink_msg_param_value_decode(&msg, &param_value);
              process_param_value_msg(param_value);
            }
            break;

            case MAVLINK_MSG_ID_ATTITUDE:  // #30
            {
              /* Message decoding: PRIMITIVE
               *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
               */
              ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving gimbal attitude.");
              mavlink_attitude_t attitude;
              mavlink_msg_attitude_decode(&msg, &attitude);
              process_attitude_msg(attitude, m_euler_ordering);
            }
            break;

            default:
              ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver < Gimbal| Receiving unhandled message #%u, ignoring.", msg.msgid);
              break;
          }
        }

        if (status.parse_error == 0)
          m_valid_chars_received++;
        const double valid_perc = 100.0 * m_valid_chars_received / m_chars_received;
        ROS_INFO_STREAM_THROTTLE(
            2.0, "[Gimbal]: Received " << m_valid_chars_received << "/" << m_chars_received << " valid characters so far (" << valid_perc << "%).");
      }
    }
    //}

    /* process_param_value_msg() method //{ */
    void process_param_value_msg(const mavlink_param_value_t& param_value)
    {
      char param_id[17];
      param_id[16] = 0;
      std::copy_n(std::begin(param_value.param_id), 16, param_id);
    
      if (std::strcmp(param_id, EULER_ORDER_PARAM_ID) == 0)
      {
        // | ----------------------- EULER_ORDER ---------------------- |
        const uint32_t value = *((uint32_t*)(&param_value.param_value));
        if (value >= 0 && value < static_cast<uint32_t>(euler_order_t::unknown))
        {
          m_euler_ordering = static_cast<euler_order_t>(value);
          ROS_INFO_THROTTLE(1.0, "[Gimbal]: %s parameter is %d (raw is %f, type is %u).", EULER_ORDER_PARAM_ID, (int)m_euler_ordering, param_value.param_value, param_value.param_type);
        }
        else
        {
          ROS_ERROR("[Gimbal]: Unknown value of %s received: %f (type: %u), ignoring.", EULER_ORDER_PARAM_ID, param_value.param_value, param_value.param_type);
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
    void process_attitude_msg(const mavlink_attitude_t& attitude, const euler_order_t& euler_order)
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

    std::string m_base_frame_id;
    std::string m_child_frame_id;

    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------
    ros::Time m_last_sent_time = ros::Time::now();
    bool m_is_connected = false;

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
