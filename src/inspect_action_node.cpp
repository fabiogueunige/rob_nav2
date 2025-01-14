/*
inspection node
Publisher: /cmd_vel
Subscriber: /aruco_markers
Service: /marker_ids
*/
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rob_nav2/srv/marker_ids.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

#define MAX_ARUCO_IDS 4
class Inspect : public plansys2::ActionExecutorClient
{
public:
  Inspect()
      : plansys2::ActionExecutorClient("inspect", 1s)
  {
    using namespace std::placeholders;
    aruco_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/aruco_markers",
        10,
        std::bind(&Inspect::aruco_callback, this, _1));

    // initialization of the publisher for the index of the marker with the lowest id
    lowest_id_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/lowest_id_at", 10);

    // index of the marker with the lowest id
    index_ = 0;
  }

  void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
  {
    int skip_aruco = 0;
    // control that id is not already in the vector
    for (int i = 0; i < marker_ids_.size(); i++)
    {
      if (marker_ids_[i] == msg->marker_ids.back())
      {
        // if the id is already in the vector, skip it
        skip_aruco = 1;
      }
    }
    // if the id is not already in the vector, add it
    if (skip_aruco == 0)
    {
      marker_ids_.push_back(msg->marker_ids.back());
      // find lowest id
      for (int j = index_; j < marker_ids_.size(); j++)
      {
        if (marker_ids_[j] < marker_ids_[index_])
        {
          // index of the marker with the lowest id
          index_ = j;
          // publish the index
          auto message = std_msgs::msg::Int32();
          message.data = index_;
          lowest_id_publisher_->publish(message);
        }
      }
      // set progress to 1.0 to finish the action
      progress_ = 1.0;
    }
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    progress_ = 0.0;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    cmd_vel_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if (progress_ < 1.0)
    {

      send_feedback(progress_, "aligning running");

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.8;

      cmd_vel_pub_->publish(cmd);
    }
    else
    {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.0;

      cmd_vel_pub_->publish(cmd);

      finish(true, progress_, "Inspection completed");
    }
  }

  float progress_;

  rclcpp::Service<rob_nav2::srv::MarkerIds>::SharedPtr marker_ids_service_;
  // vector of aruco ids
  std::vector<int> marker_ids_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // publisher for the index of the marker with the lowest id
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lowest_id_publisher_;
  // index of the marker with the lowest id
  int index_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Inspect>();

  node->set_parameter(rclcpp::Parameter("action_name", "inspect"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
