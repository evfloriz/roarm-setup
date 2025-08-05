#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ncurses.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
        : Node("teleop_publisher") {
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("/roarm/teleop_str", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&TeleopNode::timer_callback, this));

        //joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pos_cont/commands", 10);
        //joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/vel_cont/commands", 10);

        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

        // Init ncurses
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
    }

    ~TeleopNode() {
        // Close ncurses
        endwin();
    }

private:
    void timer_callback() {
        char c = getch();
        if (c == ERR) {
            return;
        }

        clear();
        printw("%c", c);
        refresh();
        
        switch (c) {
        case 'u':
            joint_positions[0] = 0.0;
            break;
        case 'j':
            joint_positions[0] = 0.5;
            break;
        case 'i':
            joint_positions[1] = 0.0;
            break;
        case 'k':
            joint_positions[1] = 0.5;
            break;
        case 'o':
            joint_positions[2] = 0.0;
            break;
        case 'l':
            joint_positions[2] = 0.5;
            break;
        case 'p':
            joint_positions[3] = 0.0;
            break;
        case ';':
            joint_positions[3] = 0.5;
            break;
        default:
            break;
        }

        auto message = std_msgs::msg::String();
        std::string input = {c};
        message.data = "Input: " + input;
        publisher_->publish(message);

        /*auto pos_message = std_msgs::msg::Float64MultiArray();
        pos_message.data = joint_positions;
        joint_publisher_->publish(pos_message);*/

        auto traj_message = trajectory_msgs::msg::JointTrajectory();
        traj_message.joint_names = joint_names;
        auto jointTrajPoint = trajectory_msgs::msg::JointTrajectoryPoint();
        jointTrajPoint.positions = joint_positions;
        traj_message.points = std::vector<trajectory_msgs::msg::JointTrajectoryPoint> {jointTrajPoint};
        joint_trajectory_publisher_->publish(traj_message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;

    std::thread keyboard_thread;
    bool stop_keyboard_thread = false;

    std::vector<double> joint_positions = {0.0, 0.0, 0.0, 0.0};
    //std::vector<double> joint_velocities = {0.0, 0.0, 0.0, 0.0};
    std::vector<std::string> joint_names = {"base_link_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_gripper_link"};
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    
    return 0;
}
