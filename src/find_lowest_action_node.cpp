/*
find lowest action node
Publisher: 
Subscriber: 
Service: 
Client: /marker_ids

*/
#include <memory>
#include <vector>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rob_nav2/srv/marker_ids.hpp"
#include "rob_nav2/srv/markers_order.hpp"


using namespace std::chrono_literals;

class FindLowest : public plansys2::ActionExecutorClient
{
public:
  FindLowest()
  : plansys2::ActionExecutorClient("find_lowest", 1s)
  {
    using namespace std::placeholders;

    markers_order_client_ = this->create_client<rob_nav2::srv::MarkersOrder>("/markers_order");
    marker_ids_client_ = this->create_client<rob_nav2::srv::MarkerIds>("/marker_ids");
    request_order = std::make_shared<rob_nav2::srv::MarkersOrder::Request>();
    request_ids = std::make_shared<rob_nav2::srv::MarkerIds::Request>();
    lowest_publisher_ = this->create_publisher<std_msgs::msg::String>("/lowest_wp", 10);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    while (!markers_order_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service orders to appear.");
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service orders to appear...");
    }
    
    // Invia la richiesta e definisci il callback per gestire la risposta
    result_order = markers_order_client_->async_send_request(request_order, 
            std::bind(&FindLowest::handle_order_response, this, std::placeholders::_1)); 

    while (!markers_ids_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service orders to appear.");
        return 1;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service orders to appear...");
    }

    result_ids = marker_ids_client_->async_send_request(request_ids, 
            std::bind(&FindLowest::handle_ids_response, this, std::placeholders::_1));

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void handle_order_response(rclcpp::Client<rob_nav2::srv::MarkersOrder>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Received order: ");
    result_order = response->order; 
  }

  void handle_ids_response(rclcpp::Client<rob_nav2::srv::MarkerIds>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Received marker IDs ");
    result_ids = response->marker_ids; 
  }

  void do_work()
  {
    index = 0;
    min_id = result_ids[0];
    for (int i = 1; i < result_ids.size(); i++)
    {
      if (result_ids[i] < min_id)
      {
        min_id = result_ids[i];
        index = i;
      }
    }

    // Sending integer after wp
    lowest_publisher_->publish(std_msgs::msg::String("wp" + std::to_string(result_order[index])));
    finish(true, index,"Lowest waypoint found");
  }

  auto request_order;
  auto request_ids;
  auto result_order;
  auto result_ids;
  int min_id;
  int index;

  rclcpp::Client<rob_nav2::srv::MarkersOrder>::SharedPtr markers_order_client_;
  rclcpp::Client<rob_nav2::srv::MarkerIds>::SharedPtr marker_ids_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lowest_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindLowest>();

  node->set_parameter(rclcpp::Parameter("action_name", "find_lowest"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

