#include <functional>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"

const float x[4] = {9.81, 7.04, 4.40, 2.72};
const float y[4] = {1.89, 10.23, 9.80, 2.72};
int i = 0;

using namespace std::placeholders;

class RouteManager : public rclcpp::Node{
public:
    using NavToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavToPose>;

    RouteManager() : Node("route_manager_node"){
        this->action_client = rclcpp_action::create_client<NavToPose>(this,"navigate_to_pose");
        this->clear_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");

        //this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
         //   std::bind(&RouteManager::run,this));
        this->run();
    }   

    void run(){
        //this->timer_->cancel();
        this->send_goal();
    }

    void send_goal(){
        RCLCPP_INFO(this->get_logger(), "Creating goal msg with index %d",i);
        NavToPose::Goal goal_msg;

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        goal_msg.pose.pose.position.x = x[i];
        goal_msg.pose.pose.position.y = y[i];
        goal_msg.pose.pose.position.z = 0;

        goal_msg.pose.pose.orientation.x = 0;
        goal_msg.pose.pose.orientation.y = 0;
        goal_msg.pose.pose.orientation.z = 0;
        goal_msg.pose.pose.orientation.w = 0.3;

        if (!this->action_client->wait_for_action_server(std::chrono::seconds(10))){
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting 10 seconds");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&RouteManager::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&RouteManager::goal_feedback_callback,this,_1,_2);
        send_goal_options.result_callback = 
            std::bind(&RouteManager::result_goal_callback,this,_1);
        this->action_client->async_send_goal(goal_msg,send_goal_options);
    }

private:
    rclcpp_action::Client<NavToPose>::SharedPtr action_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_client;
    //rclcpp::TimerBase::SharedPtr timer_;
    
    void goal_response_callback(std::shared_future<GoalHandleNav::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        else
            RCLCPP_INFO(this->get_logger(), "Goal was accepted, waiting for result");
    }

    void goal_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavToPose::Feedback> feedback)
    {
        float dist = feedback->distance_remaining;
        RCLCPP_INFO(this->get_logger(),"Distance remaining: %f",dist);
    }

    void result_goal_callback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted!");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was cancelled!");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Navigation succeded! Setting new goal!");

        i = i + 1;
        if(i == 4)
            i = 0;

        RCLCPP_INFO(this->get_logger(),"Sleeping for 5 seconds before going to next room...");
        rclcpp::sleep_for(std::chrono::seconds(5));
        this->send_goal();
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RouteManager>());
    rclcpp::shutdown();
    return 0;
}
