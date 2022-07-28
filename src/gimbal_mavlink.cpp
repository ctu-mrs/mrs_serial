#include "gimbal_mavlink.h"
#include <mavconn/interface.h>

/* #include <mavlink/v1.0/mavlink_types.h> */
/* #include <mavlink/v1.0/common/mavlink_msg_command_long.h> */
#include <mavlink/v2.0/common/mavlink_msg_gimbal_device_set_attitude.hpp>

namespace gimbal
{

    /* onInit() //{ */

    void Gimbal::onInit()
    {
      // Get paramters
      m_nh = ros::NodeHandle("~");

      ros::Time::waitForValid();

      mrs_lib::ParamLoader pl(m_nh);

      const auto url = pl.loadParam2<std::string>("url");

      if (!pl.loadedSuccessfully())
      {
        ROS_ERROR("[Gimbal]: Some compulsory parameters could not be loaded! Ending.");
        ros::shutdown();
        return;
      }

      mavconn::MAVConnInterface::Ptr client;
      try
      {
        client = mavconn::MAVConnInterface::open_url(url);
      }
      catch (mavconn::DeviceError& e)
      {
        std::cout << e.what() << std::endl;
        return;
      }

      ROS_INFO("[]: Opened MavLink!");

      mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE msg;
      msg.target_system = m_gimbal_system_id;
      msg.target_component = m_gimbal_component_id;
      msg.flags = 0;
      msg.q = {0.0, 0.0, 0.0, 1.0};
      msg.angular_velocity_x = NaN;
      client->send_message(msg);
    }
//}

}  // namespace gimbal

PLUGINLIB_EXPORT_CLASS(gimbal::Gimbal, nodelet::Nodelet)

