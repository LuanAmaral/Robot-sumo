#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <functional>


using std::placeholders::_1;
using namespace std::chrono_literals;

enum class Winner
{
    NONE,
    ROBOT_1,
    ROBOT_2
};


class check_winner: public rclcpp::Node
{
private:
    /* data */
    Winner winner = Winner::NONE;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subs_robot_1;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subs_robot_2;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;


    bool check_out_of_ring(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        auto x = msg->position.x;
        auto y = msg->position.y;
        if ( x*x + y*y > 1.5*1.5)
        {
            return true;
        }
        return false;
    }

    void callback_robot_1(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (this->check_out_of_ring(msg) && winner == Winner::NONE)
        {
            winner = Winner::ROBOT_2;
        }
    }

    void callback_robot_2(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (this->check_out_of_ring(msg) && winner == Winner::NONE)
        {
            winner = Winner::ROBOT_1;
        }
    }

    void publish_winner()
    {
        auto msg = std_msgs::msg::String();
        if (winner == Winner::NONE)
        {
            msg.data = "NONE";
            RCLCPP_INFO(this->get_logger(), "No winner yet");
        }
        else if (winner == Winner::ROBOT_1)
        {
            msg.data = "ROBOT_1";
            RCLCPP_INFO(this->get_logger(), "Robot 1 is the winner");
        }
        else if (winner == Winner::ROBOT_2)
        {
            msg.data = "ROBOT_2";
            RCLCPP_INFO(this->get_logger(), "Robot 2 is the winner");
        }
        publisher->publish(msg);
    }

public:
    check_winner(/* args */): Node("check_winner"), winner(Winner::NONE) 
    {
        subs_robot_1 = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/robot_1/pose", 10, std::bind(&check_winner::callback_robot_1, this, _1));
        subs_robot_2 = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/robot_2/pose", 10, std::bind(&check_winner::callback_robot_2, this, _1));
        publisher = this->create_publisher<std_msgs::msg::String>("/winner", 10);
        timer = this->create_wall_timer(
            100ms, std::bind(&check_winner::publish_winner, this));
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<check_winner>());
    rclcpp::shutdown();
    return 0;
}


