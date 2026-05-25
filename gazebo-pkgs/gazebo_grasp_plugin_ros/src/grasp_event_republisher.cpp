#include <gazebo/transport/transport.hh>

#include <gazebo_grasp_plugin/msgs/grasp_event.pb.h>
#include <gazebo_grasp_plugin_ros/msg/gazebo_grasp_event.hpp>
#include <rclcpp/rclcpp.hpp>

using GraspEventPtr = boost::shared_ptr<const gazebo::msgs::GraspEvent>;

rclcpp::Publisher<gazebo_grasp_plugin_ros::msg::GazeboGraspEvent>::SharedPtr gGraspEventPublisher;
rclcpp::Node::SharedPtr gNode;

void ReceiveGraspMsg(const GraspEventPtr& gzMsg)
{
  RCLCPP_INFO(gNode->get_logger(), "Re-publishing grasp event: %s",
              gzMsg->DebugString().c_str());
  gazebo_grasp_plugin_ros::msg::GazeboGraspEvent rosMsg;
  rosMsg.arm      = gzMsg->arm();
  rosMsg.object   = gzMsg->object();
  rosMsg.attached = gzMsg->attached();
  gGraspEventPublisher->publish(rosMsg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  gNode = rclcpp::Node::make_shared("gazebo_grasp_plugin_event_republisher");

  if (!gazebo::transport::init())
  {
    RCLCPP_ERROR(gNode->get_logger(),
                 "Unable to initialize gazebo transport - is gazebo running?");
    return 1;
  }
  gazebo::transport::run();

  gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
  gzNode->Init();
  gazebo::transport::SubscriberPtr subscriber;
  try
  {
    subscriber = gzNode->Subscribe("~/grasp_events", &ReceiveGraspMsg);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(gNode->get_logger(), "Error subscribing to topic: %s", e.what());
    return 1;
  }

  const std::string pubTopic = "grasp_events";
  gGraspEventPublisher =
    gNode->create_publisher<gazebo_grasp_plugin_ros::msg::GazeboGraspEvent>(pubTopic, 1);

  rclcpp::spin(gNode);
  gazebo::transport::fini();
  return 0;
}
