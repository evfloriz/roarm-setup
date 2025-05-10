#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ncurses.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
        : Node("teleop_publisher") {
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&TeleopNode::timer_callback, this));

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

        auto message = std_msgs::msg::String();
        std::string input = {c};
        message.data = "Input: " + input;
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::thread keyboard_thread;
    bool stop_keyboard_thread = false;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    
    return 0;
}
