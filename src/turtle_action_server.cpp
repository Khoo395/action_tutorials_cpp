#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/action/turtle_move.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1; 

namespace action_tutorials_cpp
{
class TurtleActionServer : public rclcpp::Node 
{
public:
using TurtleMove= my_robot_interfaces::action::TurtleMove;
using GoalHandleTurtleMove = rclcpp_action::ServerGoalHandle<TurtleMove>;
    
    explicit TurtleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("Turtle_Action_Server",options) 

    {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10); 
    subscriber = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleActionServer::UpdatePosition,this,_1));
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<TurtleMove>(this, "turtle_move", 
    std::bind(&TurtleActionServer::handle_goal,this, _1,_2),std::bind(&TurtleActionServer::handle_cancel,this, _1), std::bind(&TurtleActionServer::handle_accepted,this, _1));

    }

private:
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher; 
rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber; 
std::array<double,2> turtle_position;
rclcpp_action::Server<TurtleMove>::SharedPtr action_server_; 

void UpdatePosition(const turtlesim::msg::Pose::SharedPtr pose){
    turtle_position[0] = pose->x;
    turtle_position[1] = pose->y;
}
rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TurtleMove::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Goal Accepted with linear velocity %f and angular velovity %f",goal->linear_vel,goal->angular_vel);
    (void)uuid; //Cuz the parameter uuid isn't used 
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 

}
rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTurtleMove> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void handle_accepted(const std::shared_ptr<GoalHandleTurtleMove> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&TurtleActionServer::execute, this, _1), goal_handle}.detach();

}

bool check_clash_wall()
{
    return(turtle_position[0] < 0.1 || turtle_position[0]>11 || turtle_position[1] < 0.1 || turtle_position[1]>11); 
}

void execute(const std::shared_ptr<GoalHandleTurtleMove> goal_handle)
{ 
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TurtleMove::Feedback>();
    auto result = std::make_shared<TurtleMove::Result>();
    geometry_msgs::msg::Twist twist; 
    twist.linear.x = goal->linear_vel;
    twist.angular.z = goal->angular_vel;
    

    for (int i = 1; i<5 && rclcpp::ok(); ++i) {
     // Check if there is a cancel request
     if (goal_handle->is_canceling()){
       result->position = turtle_position;
       goal_handle->canceled(result);
       RCLCPP_INFO(this->get_logger(), "Goal canceled");
       return;
    }

     if (check_clash_wall()){
       result->position = turtle_position;
       goal_handle->abort(result);
       RCLCPP_INFO(this->get_logger(), "Goal aborted");
       return;
    }

     publisher->publish(twist);
     feedback->progress = i*25;
     goal_handle->publish_feedback(feedback);
     RCLCPP_INFO(this->get_logger(), "Publish feedback");
 
     loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->position = turtle_position;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");

}
}

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::TurtleActionServer)
