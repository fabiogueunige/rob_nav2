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
    lowest_id_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/lowest_wp",
        10,
        std::bind(&PlanController::lowestIdCallback, this, std::placeholders::_1));

      lowest_wp_ = "err";
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
    // problem_expert_->addPredicate(plansys2::Predicate("(all_inspected franka)")); 
  }

  void step()
  {
    switch (state_)
    {
    case STARTING:
    {
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
          std::cout << "Inspected wp1 " << std::endl;

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
          // Cleaning up
          problem_expert_->removePredicate(plansys2::Predicate("(visited franka wp1)"));

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal wp1 " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
          std::cout << "Successful inspected wp2 " << std::endl;

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp3))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal wp2 " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
          // Cleaning up
          problem_expert_->removePredicate(plansys2::Predicate("(visited franka wp2)"));

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal wp2" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
          std::cout << "Successful inspected wp3" << std::endl;

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(inspected franka wp4))"));

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to reach goal wp4" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
          
          // Cleaning up
          problem_expert_->removePredicate(plansys2::Predicate("(visited franka wp3)"));

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal wp3" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
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
          std::cout << "Successful inspected wp4 " << std::endl;
          
          // To go to lowest waypoint
          problem_expert_->addPredicate(plansys2::Predicate("(all_inspected franka)")); 

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(lowest_found franka))"));
          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to find lowest goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            state_ = FIND_LOWEST;
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

          // Cleaning up
          problem_expert_->removePredicate(plansys2::Predicate("(visited franka wp4)"));

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Unsuccessful replan attempt to reach goal wp4" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    }
    break;
    case FIND_LOWEST:
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
          std::cout << "Successful finished " << std::endl;

          if (lowest_wp_ == "wp1" || lowest_wp_ == "wp2" || lowest_wp_ == "wp3" || lowest_wp_ == "wp4")
          {
            std::cout << "Lowest waypoint found: " << lowest_wp_ << std::endl;
          }
          else
          {
            lowest_wp_ = "err";
          }
          std::cout << lowest_wp_ << std::endl;

          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(robot_at franka " + lowest_wp_ + "))"));
          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value())
          {
            std::cout << "Could not find plan to find lowest waypoint " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value()))
          {
            // Loop to WP1
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
    FIND_LOWEST,
    TO_LOWEST
  } StateType;
  StateType state_;

  void lowestIdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "Received lowest ID: " << std::endl;
    RCLCPP_INFO(this->get_logger(), "Received lowest ID: ");
    lowest_wp_ = msg->data;
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lowest_id_subscriber_;

  std::string lowest_wp_;
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
