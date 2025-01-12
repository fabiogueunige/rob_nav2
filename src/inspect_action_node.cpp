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

    marker_ids_service_ = create_service<rob_nav2::srv::MarkerIds>(
      "/marker_ids",
      std::bind(&Inspect::handle_service, this, _1, _2));
  }

  void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg_marker)
  {
    int tmp = msg_marker->marker_ids.back();
    for (int i = 0; i < MAX_ARUCO_IDS; i++)
    {
      if (tmp == aruco_ids_ [i])
      {
        return ;
      }           
    }
    aruco_ids_.push_back(tmp);
    progress_ = 1.0;
    // do_work();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    progress_ = 0.0;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void handle_service(
    const std::shared_ptr<rob_nav2::srv::MarkerIds::Request> /*request*/,
    std::shared_ptr<rob_nav2::srv::MarkerIds::Response> response)
    {
      response->marker_ids = aruco_ids_;
    }



  void do_work()
  {
    if (progress_ < 1.0) {

      send_feedback(progress_, "aligning running");

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.5;

      cmd_vel_pub_->publish(cmd);
    } else {
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

  std::vector<int> aruco_ids_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Inspect>();

  node->set_parameter(rclcpp::Parameter("action_name", "inspect"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

