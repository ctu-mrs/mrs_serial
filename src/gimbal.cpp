#include "gimbal.hpp"

namespace gimbal {
    /* onInit() //{ */

    void Gimbal::onInit() {
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

        if (m_stream_request_ids.size() != m_stream_request_rates.size()) {
            ROS_ERROR(
                    "[Gimbal]: Number of requested stream IDs has to be the same as the number of rates! Ending.");
            ros::shutdown();
            return;
        }

        if (!m_mount_config.from_rosparam(m_nh)) {
            ROS_ERROR("[Gimbal]: Failed to load mount configuration parameters! Ending.");
            ros::shutdown();
            return;
        }

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[Gimbal]: Some compulsory parameters could not be loaded! Ending.");
            ros::shutdown();
            return;
        }

        reconfigure_server.reset(new ReconfigureServer(m_config_mutex, m_nh));
        ReconfigureServer::CallbackType f = boost::bind(&Gimbal::callbackDynamicReconfigure, this, _1, _2);

        reconfigure_server->setCallback(f);
        // Output loaded parameters to console for double checking
        ROS_INFO_THROTTLE(1.0, "[%s] is up and running with the following parameters:",
                          ros::this_node::getName().c_str());
        ROS_INFO_THROTTLE(1.0, "[%s] portname: %s", ros::this_node::getName().c_str(), m_portname.c_str());
        ROS_INFO_THROTTLE(1.0, "[%s] baudrate: %i", ros::this_node::getName().c_str(), m_baudrate);

        const bool connected = connect();
        if (connected) {
            m_tim_sending = m_nh.createTimer(m_heartbeat_period, &Gimbal::sending_loop, this);
            m_tim_receiving = m_nh.createTimer(ros::Duration(0.05), &Gimbal::receiving_loop, this);

            m_pub_attitude = m_nh.advertise<nav_msgs::Odometry>("attitude_out", 10);
            m_pub_command = m_nh.advertise<nav_msgs::Odometry>("current_setpoint", 10);

            m_sub_attitude = m_nh.subscribe("attitude_in", 10, &Gimbal::attitude_cbk, this);
            m_sub_command = m_nh.subscribe("cmd_orientation", 10, &Gimbal::cmd_orientation_cbk, this);
            m_sub_pry = m_nh.subscribe("cmd_pry", 10, &Gimbal::cmd_pry_cbk, this);

            m_transformer = mrs_lib::Transformer("Gimbal", m_uav_name);
            sbgc_parser.init(&m_serial_port);

        } else {
            ROS_ERROR("[Gimbal]: Could not connect to the serial port! Ending.");
            ros::shutdown();
            return;
        }
    }
//}
    /* connect() //{ */

    bool Gimbal::connect() {
        ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

        if (!m_serial_port.connect(m_portname, m_baudrate)) {
            ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
            m_is_connected = false;
        } else {
            ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
            m_is_connected = true;
        }

        return m_is_connected;
    }
    // } connect

    /* request_data() method //{ */
    bool Gimbal::request_data(const uint32_t request_data_flags) {
        SBGC_cmd_data_stream_interval_t c = {0};
        ROS_INFO("[Gimbal]: Requesting data.");
        c.cmd_id = SBGC_CMD_REALTIME_DATA_CUSTOM;
        c.interval = 1;
        c.config.cmd_realtime_data_custom.flags = request_data_flags;
        c.sync_to_data = true;
        return SBGC_cmd_data_stream_interval_send(c, Gimbal::sbgc_parser);
    }
    //}

    /* sending_loop() //{ */
    void Gimbal::sending_loop([[maybe_unused]] const ros::TimerEvent &evt) {
        request_data(m_request_data_flags);
    }
    //}

    /* receiving_loop() method //{ */
    void Gimbal::receiving_loop([[maybe_unused]] const ros::TimerEvent &evt) {
        while (sbgc_parser.read_cmd()) {
            SerialCommand cmd = sbgc_parser.in_cmd;
            m_msgs_received++;
            switch (cmd.id) {
                case SBGC_CMD_REALTIME_DATA_CUSTOM: {
                    SBGC_cmd_realtime_data_custom_t msg = {0};
                    if (SBGC_cmd_realtime_data_custom_unpack(msg, m_request_data_flags, cmd) == 0) {
                        ROS_INFO_THROTTLE(2.0, "[Gimbal]: Received realtime custom data.");
                        process_custom_data_msg(msg);
                    } else {
                        ROS_ERROR_THROTTLE(2.0,
                                           "[Gimbal]: Received realtime custom data, but failed to unpack (parsed %u/%u bytes)!",
                                           cmd.pos, cmd.len);
                    }
                }
                    break;
                default:
                    ROS_INFO_STREAM_THROTTLE(2.0, "[Gimbal]: Received command ID" << (int) cmd.id << ".");
                    break;
            }

            m_valid_msgs_received = m_msgs_received - sbgc_parser.get_parse_error_count();
            const double valid_perc = 100.0 * m_valid_msgs_received / m_msgs_received;
            ROS_INFO_STREAM_THROTTLE(
                    2.0, "[Gimbal]: Received " << m_valid_msgs_received << "/" << m_msgs_received
                                               << " valid messages so far (" << valid_perc << "%).");
        }
    }
    //}

    /* attitude_cbk() method //{ */
    void Gimbal::attitude_cbk(const nav_msgs::Odometry::ConstPtr &odometry_in) {
        ROS_INFO("[Gimbal]: attitude callback");
        const geometry_msgs::Quaternion orientation_quat = odometry_in->pose.pose.orientation;
        const mat3_t rot_mat(
                quat_t(orientation_quat.w, orientation_quat.x, orientation_quat.y, orientation_quat.z));

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

    /* cmd_orientation_cbk() method //{ */
    void Gimbal::cmd_orientation_cbk(const geometry_msgs::QuaternionStamped::ConstPtr &cmd_orientation) {
        const auto ori_opt = m_transformer.transformSingle(m_stabilization_frame_id, cmd_orientation);
        if (!ori_opt.has_value()) {
            ROS_ERROR_THROTTLE(1.0,
                               "[Gimbal]: Could not transform commanded orientation from frame %s to %s, ignoring.",
                               cmd_orientation->header.frame_id.c_str(), m_stabilization_frame_id.c_str());
            return;
        }
        const geometry_msgs::Quaternion orientation_quat = ori_opt.value()->quaternion;
        const mat3_t rot_mat(
                quat_t(orientation_quat.w, orientation_quat.x, orientation_quat.y, orientation_quat.z));
        // TODO: fix...
        const vec3_t PRY_angles = rot_mat.eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX);
        const double pitch = PRY_angles.x();
        const double roll = PRY_angles.y();
        const double yaw = PRY_angles.z();
        command_mount(pitch, roll, yaw);
    }
    //}

    /* cmd_pry_cbk() method //{ */
    void Gimbal::cmd_pry_cbk(const mrs_msgs::GimbalPRY::ConstPtr &cmd_pry) {
        const auto tf_opt = m_transformer.getTransform(m_base_frame_id, m_stabilization_frame_id);
        if (!tf_opt.has_value()) {
            ROS_ERROR_THROTTLE(1.0,
                               "[Gimbal]: Could not transform commanded orientation from frame %s to %s, ignoring.",
                               m_base_frame_id.c_str(), m_stabilization_frame_id.c_str());
            return;
        }
        const mat3_t rot_mat = tf_opt->getTransformEigen().rotation();
        const quat_t q = pry2quat(cmd_pry->pitch, cmd_pry->roll, cmd_pry->yaw);
        // TODO: fix...
        const vec3_t PRY_angles = (rot_mat * q).eulerAngles(YAW_IDX, PITCH_IDX, ROLL_IDX);
        const double pitch = PRY_angles.x();
        const double roll = PRY_angles.y();
        const double yaw = PRY_angles.z();
        command_mount(pitch, roll, yaw);
    }
    //}

    /* command_mount() method //{ */
    bool Gimbal::command_mount(const double pitch, const double roll, const double yaw) {
        SBGC_cmd_control_t c = {0};
        c.mode = SBGC_CONTROL_MODE_ANGLE;
        c.anglePITCH = pitch / units2rads;
        c.angleROLL = roll / units2rads;
        c.angleYAW = yaw / units2rads;
        const bool ret = SBGC_cmd_control_send(c, sbgc_parser) == 0;

        // recalculate to degrees for the display
        const auto pitch_deg = static_cast<float>(pitch / M_PI * 180.0);
        const auto roll_deg = static_cast<float>(roll / M_PI * 180.0);
        const auto yaw_deg = static_cast<float>(yaw / M_PI * 180.0);
        ROS_INFO_THROTTLE(1.0,
                          "[Gimbal]: |Driver > Gimbal| Sending mount control command (pitch: %.0fdeg, roll: %.0fdeg, yaw: %.0fdeg).",
                          pitch_deg, roll_deg, yaw_deg);

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

    /* process_param_value_msg() method //{ */
    void Gimbal::process_param_value_msg(const mavlink_param_value_t &param_value) {
        char param_id[17];
        param_id[16] = 0;
        std::copy_n(std::begin(param_value.param_id), 16, param_id);

        if (std::strcmp(param_id, EULER_ORDER_PARAM_ID) == 0) {
            // | ----------------------- EULER_ORDER ---------------------- |
            const uint32_t value = *((uint32_t *) (&param_value.param_value));
            if (value >= 0 && value < static_cast<uint32_t>(euler_order_t::unknown)) {
                m_euler_ordering = static_cast<euler_order_t>(value);
                ROS_INFO_THROTTLE(1.0, "[Gimbal]: %s parameter is %d (raw is %f, type is %u).",
                                  EULER_ORDER_PARAM_ID, (int) m_euler_ordering, param_value.param_value,
                                  param_value.param_type);
            } else {
                ROS_ERROR("[Gimbal]: Unknown value of %s received: %f (type: %u), ignoring.", EULER_ORDER_PARAM_ID,
                          param_value.param_value, param_value.param_type);
                m_euler_ordering = euler_order_t::unknown;
            }
        } else {
            ROS_DEBUG("[Gimbal]: Unhandled parameter value %s received with value %f (type: %u), ignoring.",
                      param_id, param_value.param_value, param_value.param_type);
        }
    }
    //}

    /* process_custom_data_msg() method //{ */
    void Gimbal::process_custom_data_msg(const SBGC_cmd_realtime_data_custom_t &data) {
        /* Process the gimbal stabilization frame //{ */
        if (m_request_data_flags & cmd_realtime_data_custom_flags_z_vector_h_vector) {
            // convert the data from END to NWU
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
            const quat_t q = quat_t(rot_mat) * anax_t(M_PI, vec3_t::UnitZ());

            geometry_msgs::TransformStamped tf;
            tf.header.frame_id = m_stabilized_frame_id;
            tf.header.stamp = ros::Time::now();
            tf.child_frame_id = m_stabilization_frame_id;
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            m_pub_transform.sendTransform(tf);
        }
        //}

        /* Process the gimbal orientation frame //{ */
        if (m_request_data_flags & cmd_realtime_data_custom_flags_stator_rotor_angle) {
            // convert the data to a quaterion
            const double roll = units2rads * data.stator_rotor_angle[0];
            const double pitch = -units2rads * data.stator_rotor_angle[1];
            const double yaw = units2rads * data.stator_rotor_angle[2];
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

    quat_t Gimbal::pry2quat(const double pitch, const double roll, const double yaw) {
        // TODO: this depends on the order of motors - a 2-axis PITCH-YAW gimbal (from camera to the frame) is assumed here
        // in our case PITCH_ROLL_YAW order is used (but there is no roll)
        return quat_t(anax_t(yaw, vec3_t::UnitZ()) * anax_t(pitch, vec3_t::UnitY()));
    }


    void Gimbal::start_gimbal_motors() {
        ROS_INFO("[Gimbal]: Starting motors.");
        if (SBGC_cmd_motors_on_send(sbgc_parser) == 0) {
            ROS_INFO_THROTTLE(3.0, "[Gimbal]: Successfully started motors");
        } else {
            ROS_INFO_THROTTLE(3.0, "[Gimbal]: Error starting motors");
        }
    }

    void Gimbal::stop_gimbal_motors() {
        ROS_INFO("[Gimbal]: Turning motors off.");
        if (SBGC_cmd_motors_off_send(0, sbgc_parser) == 0) {
            ROS_INFO_THROTTLE(3.0, "[Gimbal]: Successfully offed motors");
        } else {
            ROS_INFO_THROTTLE(3.0, "[Gimbal]: Error turning off motors");
        }
    }

    bool Gimbal::turn_camera_on_deg(int16_t pitch_deg, int16_t yaw_deg, int16_t pitch_speed, int16_t yaw_speed) {
        SBGC_cmd_control_ext_t c = {0};
        c.mode[PITCH_IDX] = SBGC_CONTROL_MODE_ANGLE;
        c.mode[YAW_IDX] = SBGC_CONTROL_MODE_ANGLE;

        c.data[PITCH_IDX].angle = static_cast<int16_t>(std::ceil(pitch_deg / 0.02197265625));
        c.data[YAW_IDX].angle = static_cast<int16_t>(std::ceil(yaw_deg / 0.02197265625));

        // 0 for speed means default
        c.data[PITCH_IDX].speed = pitch_speed;
        c.data[YAW_IDX].speed = yaw_speed;

        ROS_INFO("[Gimbal]: Turning gimbal on: \n\t YAW: %hd deg; \n\t PITCH: %hd deg", yaw_deg, pitch_deg);
        return SBGC_cmd_control_ext_send(c, sbgc_parser);
    }


}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet);
