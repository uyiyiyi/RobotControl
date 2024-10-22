#ifndef PID_CONTROLLER
#define PID_CONTROLLER
#include<iostream>
#include<thread>
#include<cmath>
#include <yaml-cpp/yaml.h>

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

class PID_Control
{
private:
    double dt;
    double max_v;
    double max_w;
    double Kp_pos, Ki_pos, Kd_pos;
    double Kp_theta, Ki_theta, Kd_theta;
    double prev_error, integral;

public:
    PID_Control();
    ~PID_Control();
    void p2p(std::vector<double> &point, std::vector<double> &robot_state, std::vector<double> &ctrl);
    void traj_tracking(std::vector<std::vector<double>> &traj, std::vector<double> &robot_state, std::vector<double> &ctrl);
};

PID_Control::PID_Control()
{
    std::cout << "PID" << std::endl;
    YAML::Node config = YAML::LoadFile("../params/PID_conf.yaml");
    // 读取 PID 参数
    Kp_pos = config["PID"]["Kp_pos"].as<double>();
    Ki_pos = config["PID"]["Ki_pos"].as<double>();
    Kd_pos = config["PID"]["Kd_pos"].as<double>();
    Kp_theta = config["PID"]["Kp_theta"].as<double>();
    Ki_theta = config["PID"]["Ki_theta"].as<double>();
    Kd_theta = config["PID"]["Kd_theta"].as<double>();
    max_v = config["PID"]["max_v"].as<double>();
    max_w = config["PID"]["max_w"].as<double>();

    prev_error = 0;
    integral = 0;
}

PID_Control::~PID_Control()
{
    std::cout << "PID_Control destructor called" << std::endl;
}

void PID_Control::p2p(std::vector<double> &point, std::vector<double> &robot_state, std::vector<double> &ctrl)
{
    // 目标点
    double target_x = point[0];
    double target_y = point[1];
    double target_theta = point[2]; // 假设目标点也包含方向

    // 当前机器人状态
    double robot_x = robot_state[0];
    double robot_y = robot_state[1];
    double robot_theta = robot_state[2];

    // 计算与目标点的距离
    double error_x = target_x - robot_x;
    double error_y = target_y - robot_y;
    double distance = std::hypot(error_x, error_y);

    // 计算目标方向
    double desired_theta = std::atan2(error_y, error_x);

    // 计算方向误差
    double heading_error = desired_theta - robot_theta;

    // 确保方向误差在 -π 到 π 之间
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // 计算积分项
    integral += distance; // 累加位置误差

    // 计算微分项
    double derivative = distance - prev_error; // 位置误差的变化率

    // 使用 PID 控制器计算控制指令
    double v = Kp_pos * distance + Ki_pos * integral + Kd_pos * derivative; // 线速度
    double w = Kp_theta * heading_error; // 角速度

    // 限制最大速度
    if (v > max_v) v = max_v;
    if (w > max_w) w = max_w;

    // 将计算结果存储在 ctrl 中
    ctrl[0] = v; // 线速度
    ctrl[1] = w; // 角速度

    // 更新上一次的误差
    prev_error = distance;
    // 输出调试信息（可选）
    std::cout << "Control: v = " << v << ", w = " << w << ", Heading error: " << heading_error << std::endl;
}

void PID_Control::traj_tracking(std::vector<std::vector<double>> &traj, std::vector<double> &robot_state, std::vector<double> &ctrl)
{
    static int target_idx = 0; // 当前目标点的索引
    // 获取当前目标点
    double target_x = traj[target_idx][0];
    double target_y = traj[target_idx][1];

    // 当前机器人状态
    double robot_x = robot_state[0];
    double robot_y = robot_state[1];
    double robot_theta = robot_state[2];

    // 计算与目标点的距离
    double error_x = target_x - robot_x;
    double error_y = target_y - robot_y;
    double distance = std::hypot(error_x, error_y);

    // 如果机器人距离当前目标点很近，则切换到下一个目标点
    if (distance < 0.1) {
        if (target_idx < traj.size() - 1) {
            target_idx++;
            std::cout << "Switching to next target point: " << traj[target_idx][0] << ", " << traj[target_idx][1] << std::endl;
        } else {
            std::cout << "Reached final target point." << std::endl;
            ctrl[0] = 0; // 停止运动
            ctrl[1] = 0;
            return;
        }
    }

    // 计算目标方向
    double desired_theta = std::atan2(error_y, error_x);

    // 计算方向误差
    double heading_error = desired_theta - robot_theta;

    // 确保方向误差在 -π 到 π 之间
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // 计算积分项
    integral += distance; // 累加位置误差
    integral = clamp(integral, -5.0, 5.0);

    // 计算微分项
    double derivative = distance - prev_error; // 位置误差的变化率
    derivative = clamp(derivative, -5.0, 5.0);

    // 使用 PID 控制器计算控制指令
    double v = Kp_pos * distance + Ki_pos * integral + Kd_pos * derivative; // 线速度
    double w = Kp_theta * heading_error; //+ Ki_theta * (heading_error) + Kd_theta * derivative_y; // 角速度

    // 限制最大速度
    if (v > max_v) v = max_v;
    if (w > max_w) w = max_w;

    // 将计算结果存储在 ctrl 中
    ctrl[0] = v; // 线速度
    ctrl[1] = w; // 角速度

    // 更新上一次的误差
    prev_error = distance;
    // 输出调试信息（可选）
    std::cout << "Control: v = " << v << ", w = " << w << ", Heading error: " << heading_error << ", Target Index: " << target_idx << std::endl;
}
#endif