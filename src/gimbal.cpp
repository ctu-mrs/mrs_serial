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
#include "SBGC_lib/SBGC.h"
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
      pl.loadParam("portname", m_portname);
      pl.loadParam("baudrate", m_baudrate);
      pl.loadParam("stabilization_frame_id", m_stabilization_frame_id);
      pl.loadParam("stabilized_frame_id", m_stabilized_frame_id);
      pl.loadParam("base_frame_id", m_base_frame_id);

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

        m_pub_attitude = m_nh.advertise<nav_msgs::Odometry>("attitude_out", 10);
        m_pub_command = m_nh.advertise<nav_msgs::Odometry>("current_setpoint", 10);

        m_sub_attitude = m_nh.subscribe("attitude_in", 10, &Gimbal::attitude_cbk, this);
        m_sub_command = m_nh.subscribe("cmd_orientation", 10, &Gimbal::cmd_orientation_cbk, this);
        m_sub_pry = m_nh.subscribe("cmd_pry", 10, &Gimbal::cmd_pry_cbk, this);

        m_transformer = mrs_lib::Transformer("Gimbal", m_uav_name);
        sbgc_parser.init(&m_serial_port);
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

    SBGC_Parser sbgc_parser;
    /* static constexpr uint32_t m_request_data_flags = cmd_realtime_data_custom_flags_target_angles | cmd_realtime_data_custom_flags_target_speed | cmd_realtime_data_custom_flags_stator_rotor_angle | cmd_realtime_data_custom_flags_encoder_raw24; */
    static constexpr uint32_t m_request_data_flags = cmd_realtime_data_custom_flags_z_vector_h_vector | cmd_realtime_data_custom_flags_stator_rotor_angle;
    static constexpr double units2rads = 0.02197265625 / 180.0 * M_PI;


    /* sending_loop() //{ */
    void sending_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      request_data(m_request_data_flags);
    }
    //}

    /* request_data() method //{ */
    bool request_data(const uint32_t request_data_flags)
    {
      SBGC_cmd_data_stream_interval_t c = { 0 };
      ROS_INFO("[Gimbal]: Requesting data.");
      c.cmd_id = SBGC_CMD_REALTIME_DATA_CUSTOM;
      c.interval = 1;
      c.config.cmd_realtime_data_custom.flags = request_data_flags;
      c.sync_to_data = true;
      return SBGC_cmd_data_stream_interval_send(c, sbgc_parser);
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

      /* send_attitude(pitch, roll, yaw, pitchspeed, rollspeed, yawspeed); */
      /* send_global_position_int(); */
    }
    //}

    static constexpr int ROLL_IDX = 0;
    static constexpr int PITCH_IDX = 1;
    static constexpr int YAW_IDX = 2;

    /* cmd_orientation_cbk() method //{ */
    void cmd_orientation_cbk(geometry_msgs::QuaternionStamped::ConstPtr cmd_orientation)
    {
      const auto ori_opt = m_transformer.transformSingle(m_stabilization_frame_id, cmd_orientation);
      if (!ori_opt.has_value())
      {
        ROS_ERROR_THROTTLE(1.0, "[Gimbal]: Could not transform commanded orientation from frame %s to %s, ignoring.", cmd_orientation->header.frame_id.c_str(), m_stabilization_frame_id.c_str());
        return;
      }
      const geometry_msgs::Quaternion orientation_quat = ori_opt.value()->quaternion;
      const mat3_t rot_mat(quat_t(orientation_quat.w, orientation_quat.x, orientation_quat.y, orientation_quat.z));
      // TODO: fix...
      const vec3_t PRY_angles = rot_mat.eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX);
      const double pitch = PRY_angles.x();
      const double roll = PRY_angles.y();
      const double yaw = PRY_angles.z();
      command_mount(pitch, roll, yaw);
    }
    //}

    /* cmd_pry_cbk() method //{ */
    void cmd_pry_cbk(mrs_msgs::GimbalPRY::ConstPtr cmd_pry)
    {
      const auto tf_opt = m_transformer.getTransform(m_base_frame_id, m_stabilization_frame_id);
      if (!tf_opt.has_value())
      {
        ROS_ERROR_THROTTLE(1.0, "[Gimbal]: Could not transform commanded orientation from frame %s to %s, ignoring.", m_base_frame_id.c_str(), m_stabilization_frame_id.c_str());
        return;
      }
      const mat3_t rot_mat = tf_opt->getTransformEigen().rotation();
      const quat_t q = pry2quat(cmd_pry->pitch, cmd_pry->roll, cmd_pry->yaw);
      // TODO: fix...
      const vec3_t PRY_angles = (rot_mat*q).eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX);
      const double pitch = PRY_angles.x();
      const double roll = PRY_angles.y();
      const double yaw = PRY_angles.z();
      command_mount(pitch, roll, yaw);
    }
    //}

    /* command_mount() method //{ */
    bool command_mount(const double pitch, const double roll, const double yaw)
    {
      SBGC_cmd_control_t c = {0};
      c.mode = SBGC_CONTROL_MODE_ANGLE;
      c.anglePITCH = pitch/units2rads;
      c.angleROLL = roll/units2rads;
      c.angleYAW = yaw/units2rads;
      const bool ret = SBGC_cmd_control_send(c, sbgc_parser) == 0;

      // recalculate to degrees for the display
      const float pitch_deg = static_cast<float>(pitch/M_PI*180.0);
      const float roll_deg = static_cast<float>(roll/M_PI*180.0);
      const float yaw_deg = static_cast<float>(yaw/M_PI*180.0);
      ROS_INFO_THROTTLE(1.0, "[Gimbal]: |Driver > Gimbal| Sending mount control command (pitch: %.0fdeg, roll: %.0fdeg, yaw: %.0fdeg).", pitch_deg, roll_deg, yaw_deg);

      const quat_t q = pry2quat(pitch, roll, yaw);
      nav_msgs::OdometryPtr ros_msg = boost::make_shared<nav_msgs::Odometry>();
      ros_msg->header.frame_id = m_base_frame_id;
      ros_msg->header.stamp = ros::Time::now();
      ros_msg->child_frame_id = m_stabilized_frame_id;
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
      while (sbgc_parser.read_cmd())
      {
        SerialCommand cmd = sbgc_parser.in_cmd;
        m_msgs_received++;
        switch (cmd.id)
        {
          case SBGC_CMD_REALTIME_DATA_CUSTOM:
            {
              SBGC_cmd_realtime_data_custom_t msg = {0};
              if (SBGC_cmd_realtime_data_custom_unpack(msg, m_request_data_flags, cmd) == 0)
              {
                ROS_INFO_THROTTLE(2.0, "[Gimbal]: Received realtime custom data.");
                process_custom_data_msg(msg);
              }
              else
              {
                ROS_ERROR_THROTTLE(2.0, "[Gimbal]: Received realtime custom data, but failed to unpack (parsed %u/%u bytes)!", cmd.pos, cmd.len);
              }
            }
          default:
            ROS_INFO_STREAM_THROTTLE(2.0, "[Gimbal]: Received command ID" << (int)cmd.id << ".");
            break;
        }

        m_valid_msgs_received = m_msgs_received - sbgc_parser.get_parse_error_count();
        const double valid_perc = 100.0 * m_valid_msgs_received / m_msgs_received;
        ROS_INFO_STREAM_THROTTLE(
            2.0, "[Gimbal]: Received " << m_valid_msgs_received << "/" << m_msgs_received << " valid messages so far (" << valid_perc << "%).");
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

    quat_t pry2quat(const double pitch, const double roll, const double yaw)
    {
      // TODO: this depends on the order of motors - a 2-axis PITCH-YAW gimbal (from camera to the frame) is assumed here
      return quat_t( anax_t(yaw, vec3_t::UnitZ()) * anax_t(pitch, vec3_t::UnitY()) );
    }

    /* process_custom_data_msg() method //{ */
    void process_custom_data_msg(const SBGC_cmd_realtime_data_custom_t& data)
    {
      /* Process the gimbal stabilization frame //{ */
      if (m_request_data_flags & cmd_realtime_data_custom_flags_z_vector_h_vector)
      {
        // convert the data from END to NED
        // a vector pointing upwards (positive Z) in the gimbal frame, expressed in a static (inertial) coordinate frame
        const vec3_t z_vector = vec3_t(-data.z_vector[1], data.z_vector[0], data.z_vector[2]).normalized();
        // a vector pointing forwards (positive X) in the gimbal frame, expressed in a static (inertial) coordinate frame
        const vec3_t x_vector = vec3_t(-data.h_vector[1], data.h_vector[0], data.h_vector[2]).normalized();
        // a vector pointing left (positive Y) in the gimbal frame, expressed in a static (inertial) coordinate frame
        const vec3_t y_vector = z_vector.cross(x_vector);
        mat3_t rot_mat;
        /* rot_mat.row(0) = x_vector; */
        /* rot_mat.row(1) = y_vector; */
        /* rot_mat.row(2) = z_vector; */
        rot_mat.col(0) = x_vector;
        rot_mat.col(1) = y_vector;
        rot_mat.col(2) = z_vector;
        // convert the rotation to quaterion
        const quat_t q(rot_mat);
      
        // publish the results
        nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
        msg->header.frame_id = m_stabilized_frame_id;
        msg->header.stamp = ros::Time::now();
        msg->child_frame_id = m_stabilization_frame_id;
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

      /* Process the gimbal orientation frame //{ */
      if (m_request_data_flags & cmd_realtime_data_custom_flags_stator_rotor_angle)
      {
        // convert the data to a quaterion
        const double roll = units2rads*data.stator_rotor_angle[0];
        const double pitch = -units2rads*data.stator_rotor_angle[1];
        const double yaw = units2rads*data.stator_rotor_angle[2];
        const quat_t q = pry2quat(pitch, roll, yaw);
      
        // publish the results
        nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
        msg->header.frame_id = m_base_frame_id;
        msg->header.stamp = ros::Time::now();
        msg->child_frame_id = m_stabilized_frame_id;
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
  };

  //}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet);
