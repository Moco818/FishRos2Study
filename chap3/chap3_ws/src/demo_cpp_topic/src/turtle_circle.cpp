#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chrono"

using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

public:
    /**
     * @brief 构造函数，初始化TurtleCircle节点
     * @param node_name 节点名称，用于ROS2节点的标识
     */
    explicit TurtleCircle(const std::string& node_name) : Node(node_name)
    {
        // 创建速度命令发布者，用于控制海龟移动
        publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // 创建定时器，定期执行回调函数来发布控制命令
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
    }

private:
        /**
         * @brief 定时器回调函数，用于发布 Twist 消息
         *
         * 该函数创建一个 Twist 类型的消息，设置线速度和角速度值，
         * 然后通过 publisher_ 发布该消息。
         *
         * @param 无
         * @return 无
         */
        void timer_callback()
        {
            // 创建 Twist 消息并设置速度值
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 1.0;
            msg.angular.z = 0.5;
            publisher_->publish(msg);
        }
};

/**
 * @brief 主函数，初始化ROS2节点并执行turtle_square节点
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码，正常退出返回0
 */
int main(int argc, char *argv[])
{
    // 初始化ROS2客户端库
    rclcpp::init(argc, argv);

    // 创建TurtleCircle类型的节点实例
    auto node = std::make_shared<TurtleCircle>("turtle_square");

    // 运行节点并处理回调函数
    rclcpp::spin(node);

    // 关闭ROS2客户端库
    rclcpp::shutdown();

    return 0;
}
