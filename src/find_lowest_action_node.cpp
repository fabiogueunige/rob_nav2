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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


using namespace std::chrono_literals;

class FindLowest : public plansys2::ActionExecutorClient
{
public:
  FindLowest()
  : plansys2::ActionExecutorClient("find_lowest", 1s)
  {
    using namespace std::placeholders;

    result_index_ = 0;
    result_val_ = 1000;
    vect_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/vect_id", 
      10, 
      std::bind(&FindLowest::vect_callback, this, _1));

    lowest_publisher_ = this->create_publisher<std_msgs::msg::String>("/lowest_wp", 10);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void vect_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 0)
    {
      std::cout<<"Empty vector received"<<std::endl;
      return;
    }
    else if (msg->data.size() == 1 || (msg->data.size() == result_ids_.size() + 1))
    {
      std::cout<<"Value in find_lowest"<<std::endl;
      result_ids_.push_back(msg->data.back());
    }
    else if (msg->data.size() == result_ids_.size() || (msg->data.size() > result_ids_.size() + 1))
    {
      std::cout<<"Same size vector received, overwriting"<<std::endl;
      result_ids_ = msg->data;
    }
    else 
    {
      std::cout<< "Received vector size is strange "<<msg->data.size()<<std::endl;
    }
    check_id();
  }

  void check_id()
  {
    for (int i = 0; i < result_ids_.size(); i++)
    {
      if (result_ids_[i] < result_val_)
      {
        result_val_ = result_ids_[i];
        result_index_ = i + 1;
      }
    }
  }

  void do_work()
  {
    if (result_ids_.size() == 0)
    {
      RCLCPP_INFO(get_logger(), "No ids received");
    }
    if (result_ids_.size() == 4)
    {
      if (result_index_ > 0 && result_index_ <= 4)
      {
        // Sending integer after wp
        std_msgs::msg::String msg;
        msg.data = "wp" + std::to_string(result_index_);
        lowest_publisher_->publish(msg);
        
        finish(true, 1.0 ,"Lowest waypoint found");
      }
    }
  }

  int32_t result_val_;
  int32_t result_index_;
  std::vector<int32_t> result_ids_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr vect_sub_;
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

