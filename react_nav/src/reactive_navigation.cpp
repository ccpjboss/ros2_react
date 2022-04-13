#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

class ReactiveController : public rclcpp::Node{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    double obstacle_distance{};
    bool robot_stopped=false; 

    rclcpp::Time rotate_start;     //Time when the rotation starts
    int rotation_orientation{};

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist calculateCommand()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        //If the robot is not near an obstacle and it is not stopped
        if(obstacle_distance > 0.5 && !this->robot_stopped){
            msg.linear.x = 1.0;
        }else{ //If the robot is near an obstacle
            int32_t nsec;
            nsec = rand() % 9;
            rclcpp::Duration rotate_time(rand()%2,nsec*exp10(8));
            if (!this->robot_stopped) //If the robot was moving
            {
                //The robot will now stop
                this->robot_stopped = true;

                //Gets the time variables for the rotation
                this->rotate_start = now();
                //rotate_time = rclcpp::Duration(rand()%2,nsec*exp10(8));

                RCLCPP_INFO(this->get_logger(),"Rotation time: %f.%ld",rotate_time.seconds(),rotate_time.nanoseconds());
                
                //Gets the rotation orientation
                this->rotation_orientation = rand()%3 +1;
            }

            //Checks if the robot has rotated for enough time
            if(now()- this->rotate_start > rotate_time)
                this->robot_stopped=false;

            //Rotation orientation
            if(this->rotation_orientation%2==0)
                msg.angular.z=2;
            else
                msg.angular.z=-2;
        }
        
        return msg;
    }
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        this->obstacle_distance = *std::min_element(msg->ranges.begin(),msg->ranges.end());
    }

public:
    ReactiveController();

    void run(){
        // Send messages in a loop
        // Calculate the command to apply
        auto topics = this->get_topic_names_and_types();
        for (const auto &n : topics)
        {
            RCLCPP_INFO(this->get_logger(),"Name: %s ",n.first.c_str());
            for (int i = 0;i<n.second.size(); i++)
            {
                if (n.second[0] == "geometry_msgs/msg/Twist")
                    RCLCPP_INFO(this->get_logger(),"T: %s -- %d",n.second[i].c_str(),i);
            }
        }
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub_->publish(msg);
    }
};

ReactiveController::ReactiveController() : Node("reactive_controller")
{
    cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/stage/cmd_vel",10);
    laser_scan_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&ReactiveController::laserCallback,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&ReactiveController::run, this));
}

int main(int argc, char *argv[])
{
    srand(time(0));

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ReactiveController>());
    rclcpp::shutdown();
    return 0;
}