#include <pid_controller.h>
#include "DiffDriveModel.h"
#include "draw.h"

void p2p(PID_Control PID_Instance)
{
    // 定义目标点 (x, y, theta)
    std::vector<double> point = {5.0, 5.0, 0.0}; // 目标位置和朝向
    // 目标点的 x 和 y 值
    double target_x = point[0];
    double target_y = point[1];

    // 定义当前机器人状态 (x, y, theta)
    // 初始化机器人模型，假设初始位置和方向为 (0, 0, 0)
    Eigen::VectorXd init_state;
    init_state << 0.0, 0.0, 0,0;
    DiffDriveModel robot(init_state);

    // 定义控制指令数组 (v, w)
    std::vector<double> ctrl(2, 0.0); // 初始化控制指令

    // 初始化图形
    plt::figure();
    plt::xlim(-1, 6); // 设置 x 轴范围
    plt::ylim(-1, 6); // 设置 y 轴范围
    plt::title("Robot Path Tracking");
    plt::xlabel("X Position");
    plt::ylabel("Y Position");

    // 循环直到达到目标点
    while (true) {
        // 获取当前机器人状态
        double robot_x, robot_y, robot_yaw;
        robot.getState(robot_x, robot_y, robot_yaw);
        // 当前机器人状态
        std::vector<double> robot_state = {robot_x, robot_y, robot_yaw};

        // 调用 p2p 函数
        PID_Instance.p2p(point, robot_state, ctrl);

        // 更新机器人状态
        robot.updateState(ctrl[0], ctrl[1]);

        // 检查是否到达目标点
        double distance_to_target = std::hypot(point[0] - robot_state[0], point[1] - robot_state[1]);
        if (distance_to_target < 0.1) { // 如果距离小于阈值，则认为已到达目标
            break;
        }

        // 输出当前状态和控制指令
        std::cout << "Current state: x = " << robot_state[0] 
                  << ", y = " << robot_state[1] 
                  << ", theta = " << robot_state[2] << std::endl;
        std::cout << "Control: v = " << ctrl[0] << ", w = " << ctrl[1] << std::endl;

        // 计算箭头的终点坐标
        double arrow_length = 0.1; // 箭头长度
        double arrow_x = robot_x + arrow_length * cos(robot_yaw);
        double arrow_y = robot_y + arrow_length * sin(robot_yaw);

        // 绘制机器人位置和目标点
        plt::clf(); // 清除上一次绘图
        plt::xlim(-4, 10);
        plt::ylim(-4, 10);
        plt::plot({target_x}, {target_y}, "ro"); // 目标点为红色
        plt::plot({robot_x}, {robot_y}, "bo"); // 机器人位置为蓝色
        // 绘制表示航向的箭头
        plt::arrow(robot_x, robot_y, arrow_x, arrow_y, "r", "k"); // 用黑色箭头表示航向
        plt::legend();
        plt::pause(0.01); // 暂停以更新图形
        
        // 添加适当的延时（可选）
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 需要 <thread> 头文件
    }

    std::cout << "Reached target!" << std::endl;
    return;
}

void tracking(PID_Control PID_Instance)
{
    // 定义一个简单的轨迹（点的集合）
    std::vector<std::vector<double>> trajectory = {
        {1.0, 0.0}, // 第一个目标点
        {1.5, 1.0}, // 第二个目标点
        {2.0, 2.0}, // 第三个目标点
        {4.0, 1.0}, // 第四个目标点
        {4.0, -2.0}  // 第五个目标点
    };

    // 定义当前机器人状态 (x, y, theta)
    // 初始化机器人模型，假设初始位置和方向为 (0, 0, 0)
    Eigen::VectorXd init_state;
    init_state << 0.0, 0.0, 0,0;
    DiffDriveModel robot(init_state);

    // 定义控制指令数组 (v, w)
    std::vector<double> ctrl(2, 0.0); // 初始化控制指令

    // 初始化图形
    plt::figure();
    plt::xlim(-1, 6); // 设置 x 轴范围
    plt::ylim(-1, 6); // 设置 y 轴范围
    plt::title("Robot Path Tracking");
    plt::xlabel("X Position");
    plt::ylabel("Y Position");

    // 绘制一次静态轨迹
    draw_trajectory(trajectory);

    // 循环直到达到目标点
    while (true) {
        // 获取当前机器人状态
        double robot_x, robot_y, robot_yaw;
        robot.getState(robot_x, robot_y, robot_yaw);
        // 当前机器人状态
        std::vector<double> robot_state = {robot_x, robot_y, robot_yaw};

        // 调用 p2p 函数
        PID_Instance.traj_tracking(trajectory, robot_state, ctrl);

        // 更新机器人状态
        robot.updateState(ctrl[0], ctrl[1]);

        // 检查是否到达目标点
        double distance_to_target = std::hypot(trajectory.back()[0] - robot_state[0], trajectory.back()[1] - robot_state[1]);
        if (distance_to_target < 0.1) { // 如果距离小于阈值，则认为已到达目标
            break;
        }

        // 输出当前状态和控制指令
        std::cout << "Current state: x = " << robot_state[0] 
                  << ", y = " << robot_state[1] 
                  << ", theta = " << robot_state[2] << std::endl;
        std::cout << "Control: v = " << ctrl[0] << ", w = " << ctrl[1] << std::endl;

        // 计算箭头的终点坐标
        double arrow_length = 0.1; // 箭头长度
        double arrow_x = robot_x + arrow_length * cos(robot_yaw);
        double arrow_y = robot_y + arrow_length * sin(robot_yaw);

        // 绘制机器人位置和目标点
        plt::clf(); // 清除上一次绘图
        draw_trajectory(trajectory); // 再次绘制轨迹
        draw_robot(robot_x, robot_y, robot_state[2]); // 更新机器人位置

        plt::xlim(-4, 10);
        plt::ylim(-4, 10);
        // 绘制表示航向的箭头
        plt::pause(0.01); // 暂停以更新图形
        
        // 添加适当的延时（可选）
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 需要 <thread> 头文件
    }

    std::cout << "Reached target!" << std::endl;
    return;
}

int main() {
    std::cout << "main" << std::endl;
    PID_Control pid_inst;
    // p2p(pid_inst);
    tracking(pid_inst);

    return 0;
}