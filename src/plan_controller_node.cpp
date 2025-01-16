#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

class PlanController : public rclcpp::Node
{
public:
  PlanController()
      : rclcpp::Node("plan_controller"), state_(STARTING)
  {
    lowest_id_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/lowest_id_at",
        10,
        std::bind(&PlanController::lowestIdCallback, this, std::placeholders::_1));
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"franka", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"home", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at franka home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(visited franka home)"));
    problem_expert_->addPredicate(plansys2::Predicate("(inspected franka home)"));
  }

  void step()
  {
    switch (state_)
    {
    case STARTING:
    {
      std::cout << "Starting Case" << std::endl;
      // Set the goal for next state
      problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp1))")); // inspected franka wp1

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (!plan.has_value())
      {
        std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        break;
      }

      // Execute the plan
      if (executor_client_->start_plan_execution(plan.value()))
      {
        state_ = PATROL_WP1;
      }
    }
    break;
    case PATROL_WP1:
    {
      std::cout << "patrol WP1 Case" << std::endl;

      auto feedback = executor_client_->getFeedBack();
      for (const auto &action_feedback : feedback.action_execution_status)
      {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;

      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          robot_at = 1;
          std::cout << "Succesful inspected WP1 " << std::endl;

          // Cleanning up
          // problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp2))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            state_ = PATROL_WP2;
          }
        }
        else
        {
          for (const auto &action_feedback : feedback.action_execution_status)
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
            {
              std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    case PATROL_WP2:
    {
      auto feedback = executor_client_->getFeedBack();

      for (const auto &action_feedback : feedback.action_execution_status)
      {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;

      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          robot_at = 2;
          std::cout << "Succesful inspected WP2 " << std::endl;

          // Cleanning up
          // problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp2)"));

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp3))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            state_ = PATROL_WP3;
          }
        }
        else
        {
          for (const auto &action_feedback : feedback.action_execution_status)
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
            {
              std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    case PATROL_WP3:
    {
      auto feedback = executor_client_->getFeedBack();

      for (const auto &action_feedback : feedback.action_execution_status)
      {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;

      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          robot_at = 3;
          std::cout << "Successful inspected WP3 " << std::endl;

          // Cleanning up
          // problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp3)"));

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp4))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            state_ = PATROL_WP4;
          }
        }
        else
        {
          for (const auto &action_feedback : feedback.action_execution_status)
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
            {
              std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    case PATROL_WP4:
    {
      // Cleanning up
      //problem_expert_->removePredicate(plansys2::Predicate("(visited franka " + lowest_wp_ + ")"));

      auto feedback = executor_client_->getFeedBack();

      for (const auto &action_feedback : feedback.action_execution_status)
      {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
      }
      std::cout << "patrol WP4 Case" << std::endl;


      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          robot_at = 4;
          std::cout << "Successful inspected WP4 " << std::endl;

          // Cleanning up
          // problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp4)"));
          // print the lowest id recived
           
          std::cout << "Lowest Id: " << lowest_id_ << std::endl;

          if (lowest_id_ == 0)
          {
            if (robot_at == 1){
              break;
            }
            problem_expert_->setGoal(plansys2::Goal("(and(robot_at franka wp1))"));
            std::cout << "goal:wp4 " << lowest_id_ << std::endl;
          }
          else if (lowest_id_ == 1)
          {
            if (robot_at == 2){
              break;
            }

            problem_expert_->setGoal(plansys2::Goal("(and(robot_at franka wp2))"));
            std::cout << "goal " << lowest_id_ << std::endl;

          }
          else if (lowest_id_ == 2)
          {
            if (robot_at == 3){
              break;
            }
            problem_expert_->setGoal(plansys2::Goal("(and(robot_at franka wp3))"));
          }
          else if (lowest_id_ == 3)
          {
            if (robot_at == 4){
              break;
            }
            problem_expert_->setGoal(plansys2::Goal("(and(robot_at franka wp4))"));
          }
          else
          {
            lowest_wp_ = "robot_at franka err";
          }
        
          // Set the goal for next state
          //problem_expert_->setGoal(plansys2::Goal("(and(lowest_wp_))"));
          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            // Loop to WP4
            state_ = TO_LOWEST;
          }
        }
        else
        {
          for (const auto &action_feedback : feedback.action_execution_status)
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
            {
              std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    case TO_LOWEST:
    {

      std::cout << "TO_LOWEST Case" << std::endl;

      auto feedback = executor_client_->getFeedBack();

      for (const auto &action_feedback : feedback.action_execution_status)
      {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;

      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          std::cout << "Successful finished " << std::endl;
          break;
        }
        else
        {
          for (const auto &action_feedback : feedback.action_execution_status)
          {
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED)
            {
              std::cout << "[" << action_feedback.action << "] finished with error: " << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    default:
      break;
    }
  }

private:
  typedef enum
  {
    STARTING,
    PATROL_WP1,
    PATROL_WP2,
    PATROL_WP3,
    PATROL_WP4,
    TO_LOWEST
  } StateType;
  StateType state_;

  void lowestIdCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received lowest ID: %d", msg->data);

    lowest_id_ = msg->data;
    std::cout << "Callback LowestId recived: " << lowest_id_ << std::endl;  
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lowest_sub_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lowest_id_subscriber_;
  int lowest_id_;
  std::string lowest_wp_;
  int robot_at = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
