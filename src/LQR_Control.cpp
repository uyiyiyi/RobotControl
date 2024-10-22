#include <iostream>
#include<thread>
#include <lqr_controller.h>
#include "DiffDriveModel.h"
#include "matplotlibcpp.h"

using namespace Eigen;
using namespace std;
namespace plt = matplotlibcpp; // 为 matplotlibcpp 命名空间简化命名

// 计算二次贝塞尔曲线
// std::vector<std::vector<double>> generateBezierCurve(const std::vector<std::vector<double>>& control_points, int num_points) {
//     std::vector<std::vector<double>> curve_points;

//     for (int i = 0; i <= num_points; ++i) {
//         double t = static_cast<double>(i) / num_points;
//         double x = (1 - t) * (1 - t) * control_points[0][0] + 
//                    2 * (1 - t) * t * control_points[1][0] + 
//                    t * t * control_points[2][0];
//         double y = (1 - t) * (1 - t) * control_points[0][1] + 
//                    2 * (1 - t) * t * control_points[1][1] + 
//                    t * t * control_points[2][1];
//         double theta = atan2(y, x); // 计算角度
//         double v = 0.5; // 假设线速度为常数

//         curve_points.push_back({x, y, theta, v});
//     }

//     return curve_points;
// }

void draw_trajectory(const std::vector<std::vector<double>> &trajectory) {
    // 将轨迹点一次性绘制
    std::vector<double> traj_x, traj_y;
    for (const auto &point : trajectory) {
        traj_x.push_back(point[0]);
        traj_y.push_back(point[1]);
    }
    plt::plot(traj_x, traj_y, "b-"); // 蓝色圆圈表示轨迹点
}

void draw_robot(double robot_x, double robot_y, double robot_theta) {
    // 绘制机器人当前位置和方向（航向角）
    double arrow_x = std::cos(robot_theta); // 箭头x方向
    double arrow_y = std::sin(robot_theta); // 箭头y方向
    
    // 绘制机器人位置为红点，箭头表示航向
    plt::plot({robot_x}, {robot_y}, "ro");
    plt::arrow(robot_x, robot_y, arrow_x, arrow_y, "r", "k"); // 黑色箭头表示航向
}

// 定义机器人状态和控制输入的维度
const int STATE_DIM = 3; // [x, y, theta]
const int CONTROL_DIM = 2; // [v, w]

// 创建轨迹点，假设轨迹包含多个点，每个点包含 [x, y, theta, v]
// std::vector<std::vector<double>> trajectory = {
//     {1.0, 0.0, 0.0, 1.0}, // 第一个目标点
//     {1.5, 1.0, 1.0, 1.0}, // 第二个目标点
//     {2.0, 2.0, 1.0, 1.5}, // 第三个目标点
//     {4.0, 1.0, -0.5, 1.0}, // 第四个目标点
//     {4.0, -2.0, -1.571, 0.5}  // 第五个目标点
// };

// 控制点
// std::vector<std::vector<double>> control_points = {
//     {0.0, 0.0},   // 起点
//     {10.0, 0.0},   // 控制点
//     {10.0, 10.0}    // 终点
// };

// 生成贝塞尔曲线轨迹
// int num_points = 400; // 生成的轨迹点数量
// auto trajectory = generateBezierCurve(control_points, num_points);

// 定义圆的半径
const double radius = 5.0;

// 生成圆的轨迹
std::vector<std::vector<double>> generateCircleTrajectory(int num_points) {
    std::vector<std::vector<double>> trajectory;

    // 循环生成轨迹点
    for (int i = 0; i < num_points; ++i) {
        double theta = 2 * M_PI * i / num_points; // 角度从 0 到 2π
        double x = radius * cos(theta);           // 计算 x 坐标
        double y = radius * sin(theta);           // 计算 y 坐标
        theta += M_PI / 2;
        double v = 2;
        // 将轨迹点添加到列表
        trajectory.push_back({x, y, theta, v});
    }

    return trajectory;
}

int main() {
    double v_max = 5;
    double w_max = M_PI;
    int num_points = 100; // 生成的轨迹点数量
    // 获取圆轨迹
    std::vector<std::vector<double>> trajectory = generateCircleTrajectory(num_points);

    // 初始化机器人模型，假设初始位置和方向为 (0, 0, 0)
    Eigen::VectorXd init_state(STATE_DIM);
    init_state << 4.5, 0, M_PI / 6 ; // [x, y, theta]

    DiffDriveModel robot(init_state);
    // 初始化机器人状态和控制指令
    VectorXd robot_state = init_state; // x, y, theta
    VectorXd Error = init_state;
    std::vector<double> ctrl = {2.0, 0.0}; // 线速度和角速度
    std::vector<double> v_theta = {0.0, 0.0};
    // 创建LQR控制器
    lqr_controller lqr;
    
    // 控制循环
    size_t target_index = 0; // 当前目标点的索引
    while (true) {
        // 获取当前目标点

        if (target_index < trajectory.size()) {
            std::vector<double> target_point = trajectory[target_index];
            
            // 更新机器人状态
            double robot_x, robot_y, robot_yaw;
            robot.getState(robot_x, robot_y, robot_yaw);

            // 计算 A 和 B 矩阵，假设 v_theta 是机器人的速度和方向
            v_theta[0] = target_point[3]; // 线速度
            v_theta[1] = target_point[2]; // 当前角度
            cout << "v_theta = " << v_theta[0] << ", " << v_theta[1] << endl;
            lqr.calAB(v_theta);
            cout << "1" << endl;
            // cout << "A:" << lqr.A_ << endl;
            // cout << "B:" << lqr.B_ << endl;
            // cout << "K:" << lqr.K_ << endl;
            // 计算 LQR 控制增益
            lqr.computeLQRGain();
            cout << "2" << endl;
            // 控制计算
            robot_state << robot_x, robot_y, robot_yaw;
            Error << robot_x - target_point[0], robot_y - target_point[1], robot_yaw - target_point[2];
            cout << "Error: " << Error << endl;
            VectorXd control_vector = lqr.control(Error);

            // 更新控制指令
            ctrl[0] += control_vector(0); // 线速度
            ctrl[1] += control_vector(1); // 角速度

            ctrl[0] = clamp(ctrl[0], -v_max, v_max);
            ctrl[1] = clamp(ctrl[1], -w_max, w_max);
            // 更新机器人状态
            robot.updateState(ctrl[0], ctrl[1]);

            // 检查是否到达目标点
            double distance_to_target = std::hypot(target_point[0] - robot_state[0], target_point[1] - robot_state[1]);
            cout << "distance_to_target = " << distance_to_target << endl;
            std::cout << "Target Point Index: " << target_index << std::endl;
            if (distance_to_target < 0.3) { // 如果距离目标点小于阈值，切换到下一个目标点
                target_index++;
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

            plt::xlim(-10, 10);
            plt::ylim(-10, 10);
            // 绘制表示航向的箭头
            plt::pause(0.01); // 暂停以更新图形
            
            // 添加适当的延时（可选）
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 需要 <thread> 头文件
        } else {
            std::cout << "已到达所有目标点，停止控制." << std::endl;
            break; // 如果已到达所有目标点，退出循环
        }

    }

    return 0;
}