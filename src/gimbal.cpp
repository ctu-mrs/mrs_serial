/* Original code copied from MAVLInk_DroneLights by Juan Pedro López */

#include <boost/smart_ptr/make_shared_object.hpp>
#include <limits>
#include <ros/package.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

#include <string>
#include <serial_port.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/param_loader.h>

#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "mavlink/mavlink.h"

namespace gimbal
{

  /* class Gimbal //{ */

  class Gimbal : public nodelet::Nodelet
  {
  private:
    enum class euler_order_t
    {
      pitch_roll_yaw = 0,
      roll_pitch_yaw = 1,
    };

    // https://mavlink.io/en/messages/common.html

    enum MAV_CMD
    {
      MAV_CMD_DO_MOUNT_CONTROL = 205,  //< Mission command to control a camera or antenna mount
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

  public:
    /* onInit() //{ */

    virtual void onInit() override
    {

      // Get paramters
      m_nh = ros::NodeHandle("~");

      ros::Time::waitForValid();

      mrs_lib::ParamLoader pl(m_nh);

      pl.loadParam("portname", m_portname);
      pl.loadParam("baudrate", m_baudrate);
      pl.loadParam("heartbeat_period", m_heartbeat_period, ros::Duration(1.0));

      pl.loadParam("driver/system_id", m_driver_system_id);
      pl.loadParam("driver/component_id", m_driver_component_id);

      pl.loadParam("gimbal/system_id", m_gimbal_system_id);
      pl.loadParam("gimbal/component_id", m_gimbal_component_id);

      pl.loadParam("stream_requests/ids", m_stream_request_ids);
      pl.loadParam("stream_requests/rates", m_stream_request_rates);

      if (m_stream_request_ids.size() != m_stream_request_rates.size())
      {
        ROS_ERROR("[Gimbal]: Number of requested stream IDs has to be the same as the number of rates! Ending.");
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

      if (!serial_port_.connect(m_portname, m_baudrate))
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
        const bool success = request_data();
        if (success)
          m_hbs_since_last_request = 0;
      }

      static float pitch = 0;
      static float roll = 0;
      static float yaw = 0;
      pitch = (float)std::fmod(pitch + 0.1, 0.3);
      yaw = (float)std::fmod(yaw + 0.1, 0.3);
      command_mount(pitch, roll, yaw);
    }
    //}

    /* request_data() method //{ */
    bool request_data()
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

      for (int it = 0; it < m_stream_request_ids.size(); it++)
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
        const int request_stream_id_loaded = m_stream_request_ids.at(it);
        if (request_stream_id_loaded > std::numeric_limits<uint8_t>::max())
        {
          ROS_ERROR("[Gimbal]: Cannot request stream ID larger than %u (got %d)!", std::numeric_limits<uint8_t>::max(), request_stream_id_loaded);
          success = false;
          continue;
        }
        const uint8_t request_stream_id = static_cast<uint8_t>(request_stream_id_loaded);

        const int request_stream_rate_loaded = m_stream_request_rates.at(it);
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
        success = success && serial_port_.sendCharArray(buf, len);
      }

      // TODO: Request the value of the "EULER_ORDER" parameter to know in what format does the attitude arrive
      /* mavlink_msg_param_value_get_param_id() */
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
      return serial_port_.sendCharArray(buf, len);
    }
    //}

    /* command_mount() method //{ */
    bool command_mount(const float pitch, const float roll, const float yaw)
    {
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      /* uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, */
      /*                                uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2,
       * float param3, float param4, float param5, float param6, float param7) */

      // MAV_CMD_DO_MOUNT_CONTROL command parameters:
      /* 1: Pitch	      pitch depending on mount mode (degrees or degrees/second depending on pitch input). */
      /* 2: Roll	      roll depending on mount mode (degrees or degrees/second depending on roll input). */
      /* 3: Yaw	        yaw depending on mount mode (degrees or degrees/second depending on yaw input). */
      /* 4: Altitude	  altitude depending on mount mode.	(m) */
      /* 5: Latitude	  latitude, set if appropriate mount mode. */
      /* 6: Longitude	  longitude, set if appropriate mount mode. */
      /* 7: Mode	      Mount mode.	(MAV_MOUNT_MODE) */
      mavlink_msg_command_long_pack(m_driver_system_id, m_driver_component_id, &msg,
                                    // tgt. system id,  tgt. component id,      command id,               confirmation
                                    m_gimbal_system_id, m_gimbal_component_id,  MAV_CMD_DO_MOUNT_CONTROL, 0,
                                    // command parameters
                                    pitch, roll, yaw, 0, 0, 0, MAV_MOUNT_MODE_MAVLINK_TARGETING);
      const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

      // send the heartbeat message
      ROS_INFO_STREAM_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending mount control command.");
      return serial_port_.sendCharArray(buf, len);
    }
    //}

    /* receiving_loop() method //{ */
    void receiving_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      mavlink_message_t msg;
      mavlink_status_t status;

      uint8_t c;
      while (serial_port_.readChar(&c))
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
              process_attitude_msg(attitude);
            }
            break;

            default:
              break;
          }
        }

        if (status.parse_state != MAVLINK_PARSE_STATE_IDLE && status.parse_state != MAVLINK_PARSE_STATE_UNINIT)
          m_valid_chars_received++;
        const double valid_perc = 100.0 * m_valid_chars_received / m_chars_received;
        ROS_INFO_STREAM_THROTTLE(
            2.0, "[Gimbal]: Received " << m_valid_chars_received << "/" << m_chars_received << " valid characters so far (" << valid_perc << "%).");
      }
    }
    //}

    /* process_attitude_msg() method //{ */
    void process_attitude_msg(const mavlink_attitude_t& attitude, const euler_order_t& euler_order = euler_order_t::pitch_roll_yaw)
    {
      using quat_t = Eigen::Quaterniond;
      using anax_t = Eigen::AngleAxisd;
      using vec3_t = Eigen::Vector3d;

      quat_t q;
      switch (euler_order)
      {
        case euler_order_t::roll_pitch_yaw:
          q = anax_t(attitude.roll, vec3_t::UnitX()) * anax_t(attitude.pitch, vec3_t::UnitY()) * anax_t(attitude.yaw, vec3_t::UnitZ());
          break;
        case euler_order_t::pitch_roll_yaw:
        default:
          q = anax_t(attitude.pitch, vec3_t::UnitY()) * anax_t(attitude.roll, vec3_t::UnitX()) * anax_t(attitude.yaw, vec3_t::UnitZ());
          break;
      }

      nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
      msg->header.frame_id = "gimbal";
      msg->header.stamp = ros::Time::now();
      msg->child_frame_id = "gimbal/camera";
      msg->pose.pose.orientation.x = q.x();
      msg->pose.pose.orientation.y = q.y();
      msg->pose.pose.orientation.z = q.z();
      msg->pose.pose.orientation.w = q.w();
      m_pub_attitude.publish(msg);
    }
    //}

    ros::NodeHandle m_nh;
    ros::Timer m_tim_sending;
    ros::Timer m_tim_receiving;
    ros::Publisher m_pub_attitude;

    serial_port::SerialPort serial_port_;

    std::string m_portname;
    int m_baudrate;
    ros::Duration m_heartbeat_period;

    int m_driver_system_id;
    int m_driver_component_id;

    int m_gimbal_system_id;
    int m_gimbal_component_id;

    std::vector<int> m_stream_request_ids;
    std::vector<int> m_stream_request_rates;

    ros::Time m_last_sent_time = ros::Time::now();
    bool m_is_connected = false;

    int m_hbs_since_last_request = 0;
    int m_hbs_request_period = 5;
    size_t m_chars_received = 0;
    size_t m_valid_chars_received = 0;
  };

  //}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet);
