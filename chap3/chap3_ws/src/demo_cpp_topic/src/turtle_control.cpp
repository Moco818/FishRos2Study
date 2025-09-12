#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleController : public rclcpp::Node
{
public:
        /**
         * @brief 构造函数，初始化TurtleController节点
         * @details 创建了一个名为"turtle_controller"的ROS2节点，并初始化了速度发布器和位姿订阅器
         * @param 无
         * @return 无
         */
        TurtleController() : Node("turtle_controller") {
            // 创建速度指令发布器，用于控制海龟的线速度和角速度
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            // 创建位姿信息订阅器，用于接收海龟当前的位置和方向信息
            pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
                std::bind(&TurtleController::on_pose_received_, this, std::placeholders::_1));
        }

private:
        /**
         * @brief 处理接收到的乌龟位置信息回调函数
         * @param pose 乌龟当前位置信息的共享指针
         * 
         * 该函数在接收到乌龟的位置信息后被调用，用于计算位置误差并发布相应的速度控制指令
         */
        void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) 
        {
        	// TODO: 收到位置计算误差，发布速度指令
            auto message = geometry_msgs::msg::Twist();
            // 1. 记录当前位置
            double current_x = pose -> x;
            double current_y = pose -> y;
            RCLCPP_INFO(this -> get_logger(), "当前位置：（x=%f, y=%f）", current_x, current_y);
            // 2. 计算与目标之间的距离，以及与当前海龟朝向的角度差
            double distance = std::sqrt((target_x_ - current_x) * (target_x_ - current_x) + (target_y_ - current_y) * (target_y_ - current_y));
            double angle = std::atan2(target_y_ - current_y, target_x_ - current_y) - pose -> theta;
            // 3. 控制策略：距离大于 0.1 继续运动，角度差大于 0.2 则原地旋转，否则直行
            if (distance > 0.1)
            {
                if (fabs(angle) > 0.2)
                {
                    message.angular.z = fabs(angle);
                }
                else
                {
                    message.linear.x = k_ * distance;
                }
                
            };
            // 4. 限制最大值并发布消息
            if (message.linear.x > max_speed_)
            {
                message.linear.x = max_speed_;
            }
            velocity_publisher_ -> publish(message);
        }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    double target_x_{1.0};      // 目标位置 x, 设置默认值 1.0
    double target_y_{1.0};      // 目标位置 y, 设置默认值 1.0
    double k_{1.0};             // 比例系数，控制输出 = 误差 × 比例系数
    double max_speed_{3.0};     // 最大线速度，设置默认值 3.0
};

/**
 * @brief 主函数，初始化ROS2节点并运行海龟控制器
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码，正常退出返回0
 */
int main(int argc, char **argv) {
    // 初始化ROS2客户端库
    rclcpp::init(argc, argv);
    
    // 创建海龟控制器节点实例
    auto node = std::make_shared<TurtleController>();
    
    // 运行节点，处理回调函数直到节点被关闭
    rclcpp::spin(node);
    
    // 关闭ROS2客户端库
    rclcpp::shutdown();
    
    return 0;
}
