#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/action/turtle_move.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include <iostream>
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{

class TurtleActionClient : public rclcpp::Node // MODIFY NAME
{
public:
using TurtleMove= my_robot_interfaces::action::TurtleMove;
using GoalHandleTurtleMove = rclcpp_action::ClientGoalHandle<TurtleMove>;
    explicit TurtleActionClient(const rclcpp::NodeOptions & options) 
    : Node("turtle_action_client", options) 
    {
        this->action_client_ = rclcpp_action::create_client<TurtleMove>(this, "turtle_move");
        std::thread(std::bind(&TurtleActionClient::read_keyboard, this)).detach(); 
    }

private:
rclcpp_action::Client<TurtleMove>::SharedPtr action_client_; 
GoalHandleTurtleMove::SharedPtr last_goal;


void send_goal(const my_robot_interfaces::action::TurtleMove::Goal goal_msg)
{   using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<TurtleMove>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TurtleActionClient::goal_response_callback, this, _1);
     send_goal_options.feedback_callback =     std::bind(&TurtleActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =        std::bind(&TurtleActionClient::result_callback, this, _1);
    this->action_client_->async_send_goal(goal_msg, send_goal_options);

}

void cancel_goal()
{
    auto cancel_future = action_client_->async_cancel_goal(last_goal); 
    auto response = cancel_future.get();
    if(response)
    {RCLCPP_INFO(this->get_logger(), "Cancel Request Rejected");}
    else{RCLCPP_INFO(this->get_logger(), "Cancel Request Accepted");}
    
}

void read_keyboard()
{RCLCPP_INFO(this->get_logger(), "Control the turtle using the standard WSAD method. ");
    while(true)
{
    char input; 
    std::cin >> input; 
    TurtleMove::Goal goal_msg; 
    if (input == 't'){
        cancel_goal();
    }
    else{
    if(input == 'w'){
        goal_msg.linear_vel = 1;
    }
    else if(input == 's'){
        goal_msg.linear_vel = -1;
    }
    else if(input == 'a'){
        goal_msg.angular_vel = -0.7;
    }
    else if(input == 'd'){
        goal_msg.angular_vel = 0.7;
    }
    send_goal(goal_msg);
    } 
}
}


void goal_response_callback(const std::shared_future<GoalHandleTurtleMove::SharedPtr> future)
{
    auto goal_handle = future.get();
    if(!goal_handle){RCLCPP_INFO(this->get_logger(), "The Goal has been Rejected");}
    else{RCLCPP_INFO(this->get_logger(), "The Goal has been Accepted");}
    last_goal = goal_handle; 
    
}

void feedback_callback(GoalHandleTurtleMove::SharedPtr, const std::shared_ptr<const TurtleMove::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "The Goal is %d% completed", feedback->progress);
}

void result_callback(const GoalHandleTurtleMove::WrappedResult & result)
{

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
         return;
      default:
         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
         return;
}
RCLCPP_INFO(this->get_logger(), "The Goal is Completed, the new location of the turtle is %f,%f", result.result->position[0],result.result->position[1]); 

}


};


}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::TurtleActionClient)
