#include <iostream>
#include<thread>
#include <lqr_controller.h>
#include "DiffDriveModel.h"
#include "draw.h"

using namespace Eigen;
using namespace std;


// 定义机器人状态和控制输入的维度
const int STATE_DIM = 3; // [x, y, theta]
const int CONTROL_DIM = 2; // [v, w]

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