/* Original code copied from MAVLInk_DroneLights by Juan Pedro López */

#include <ros/package.h>
#include <ros/ros.h>
#include <mutex>

#include <string>
#include <serial_port.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/param_loader.h>

#include "mavlink/mavlink.h"

// Mavlink variables
const int num_hbs = 60;                     // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;


void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   *
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {200};


  for (int i = 0; i < maxStreams; i++)
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
    mavlink_msg_request_data_stream_pack(1, 1, &msg, 1, 154, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
  }

  // Request: PARAM_REQUEST_LIST. Only for full log recording
  /*
   * Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
   */
  /*
    // Configure
    uint8_t system_id=2;
    uint8_t component_id=200;
    // mavlink_message_t* msg;
    uint8_t target_system=1;
    uint8_t target_component=0;

    // Pack
    mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
      target_system, target_component);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send
  #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf,len);
  #else
      Serial.write(buf, len);
  #endif
  */
}


void comm_receive()
{

  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available() > 0)
  {
    uint8_t c = Serial.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
        {
          // E.g. read GCS heartbeat and go into
          // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
          // mySerial.println("PX HB");
#endif
        }
        break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
        {
          /* Message decoding: PRIMITIVE
           *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
           */
          // mavlink_message_t* msg;
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
        }
        break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
        {
          /* Message decoding: PRIMITIVE
           *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
           */
          // mavlink_message_t* msg;
          mavlink_param_value_t param_value;
          mavlink_msg_param_value_decode(&msg, &param_value);
        }
        break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
        {
          /* Message decoding: PRIMITIVE
           *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
           */
          mavlink_raw_imu_t raw_imu;
          mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
          // mySerial.println("PX RAW IMU");
          // mySerial.println(raw_imu.xacc);
#endif
        }
        break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
        {
          /* Message decoding: PRIMITIVE
           *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
           */
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);
        }
        break;


        default:
          break;
      }
    }
  }
}


namespace gimbal
{

  /* class Gimbal //{ */

  class Gimbal : public nodelet::Nodelet
  {

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
      pl.loadParam("heartbeat_period", m_heartbeat_period);

      // Output loaded parameters to console for double checking
      ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
      ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), m_portname.c_str());
      ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), m_baudrate);

      connect();

      m_is_initialized = true;
    }
    //}

  private:
    enum serial_receiver_state
    {
      WAITING_FOR_MESSSAGE,
      EXPECTING_SIZE,
      EXPECTING_PAYLOAD,
      EXPECTING_CHECKSUM
    };

    /* connect() //{ */

    bool connect(void)
    {

      ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

      if (!serial_port_.connect(m_portname, m_baudrate))
      {
        ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
        m_is_connected = false;
        return 0;
      }

      ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
      m_is_connected = true;

      return 1;
    }

    //}

    /* sending_loop() //{ */
    void sending_loop()
    {
      // MAVLink
      /* The default UART header for your MCU */
      constexpr int sysid = 1;                  ///< ID 20 for this airplane. 1 PX, 255 ground station
      constexpr int compid = 1;                 ///< The component sending the message
      constexpr int type = MAV_TYPE_QUADROTOR;  ///< This system is an airplane / fixed wing
    
      // Define the system type, in this case an airplane -> on-board controller
      constexpr uint8_t system_type = MAV_TYPE_GENERIC;
      constexpr uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    
      constexpr uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
      constexpr uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
      constexpr uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    
      // Initialize the required buffers
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
      // Pack the message
      mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    
      // Copy the message to the send buffer
      const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
      // Send the message with the standard UART send function
      // uart0_send might be named differently depending on
      // the individual microcontroller / library in use.
      const ros::Time cur_time = ros::Time::now();
      if (cur_time - m_last_sent_time >= m_heartbeat_period)
      {
        m_last_sent_time = cur_time;
    
        Serial.write(buf, len);
    
        // Mav_Request_Data();
        num_hbs_pasados++;
        if (num_hbs_pasados >= num_hbs)
        {
          // Request streams from Pixhawk
          Mav_Request_Data();
          num_hbs_pasados = 0;
        }
      }
    
      // Check reception buffer
      comm_receive();
    }
    //}

    ros::NodeHandle m_nh;

    serial_port::SerialPort serial_port_;

    std::string m_portname;
    int m_baudrate;
    std::string m_uav_name;
    ros::Duration m_heartbeat_period;

    ros::Time m_last_sent_time = ros::Time::now();
    bool m_is_connected = false;
    bool m_is_initialized = false;
  };

  //}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(servo::Servo, nodelet::Nodelet);
