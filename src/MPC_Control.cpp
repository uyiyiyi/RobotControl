#include "mpc_controller.h"
#include "draw.h"



int main() {
    int num_points = 100; // 生成的轨迹点数量
    double v_max = 2;
    double w_max = M_PI;
    // 获取圆轨迹
    std::vector<std::vector<double>> trajectory = generateCircleTrajectory(num_points);

    double dt = 0.01;
    int N = 10;
    int state_dim = 3;
    int control_dim = 2;
    // 初始化MPC控制器
    mpc_controller mpc(dt, N, state_dim, control_dim);

    // // 定义差分驱动机器人模型的初始状态
    VectorXd init_state(state_dim);
    init_state << 4.9, 0, M_PI / 2; // 初始状态 x=0, y=0, theta=0
    VectorXd current_state(state_dim), X_ref(state_dim * N);
    double robot_x, robot_y, robot_yaw;
    std::vector<double> ctrl = {2.0, 0.0}; // 线速度和角速度
    std::vector<double> v_theta = {0, 0};
    
    DiffDriveModel robot(init_state); // 使用已有的差分驱动模型
    size_t target_index = 0; // 当前目标点的索引
    // 控制循环
    while (true) {
        if (target_index < trajectory.size() - N) {
            std::vector<double> target_point = trajectory[target_index];
            // 获取机器人当前状态
            robot.getState(robot_x, robot_y, robot_yaw); // 假设这个函数存在
            current_state << robot_x, robot_y, robot_yaw;
            cout << "current_state: " << current_state << endl;
            for (size_t i = 0; i < N; i++)
            {
                X_ref(3 * i) = trajectory[i + target_index][0];
                X_ref(3 * i + 1) = trajectory[i + target_index][1];
                X_ref(3 * i + 2) = trajectory[i + target_index][2];
            }

            v_theta[0] = ctrl[0];
            v_theta[1] = robot_yaw;
            mpc.calABQRHf(v_theta, current_state, X_ref);
            // 求解MPC问题，获取控制输入
            VectorXd optimal_control = mpc.solveQP();
            

            // 更新控制指令
            ctrl[0] = optimal_control(0);
            ctrl[1] = optimal_control(1);
            ctrl[0] = clamp(ctrl[0], -v_max, v_max);
            ctrl[1] = clamp(ctrl[1], -w_max, w_max);
            
            // 使用控制输入更新机器人状态
            robot.updateState(ctrl[0], ctrl[1]); // 假设此函数更新机器人状态
            // 检查是否到达目标点
            double distance_to_target = std::hypot(target_point[0] - robot_x, target_point[1] - robot_y);
            cout << "distance_to_target = " << distance_to_target << endl;
            cout << "Target Point Index: " << target_index << endl;
            if (distance_to_target < 0.4) { // 如果距离目标点小于阈值，切换到下一个目标点
                target_index++;
            }

            // 输出当前状态
            std::cout << "Current State: " << current_state.transpose() << std::endl;

            // 输出当前状态和控制指令
            std::cout << "Current state: x = " << robot_x
                    << ", y = " << robot_y
                    << ", theta = " << robot_yaw << std::endl;
            std::cout << "Control: v = " << ctrl[0] << ", w = " << ctrl[1] << std::endl;

            // 计算箭头的终点坐标
            double arrow_length = 0.1; // 箭头长度
            double arrow_x = robot_x + arrow_length * cos(robot_yaw);
            double arrow_y = robot_y + arrow_length * sin(robot_yaw);

            // 绘制机器人位置和目标点
            plt::clf(); // 清除上一次绘图
            draw_trajectory(trajectory); // 再次绘制轨迹
            draw_robot(robot_x, robot_y, robot_yaw); // 更新机器人位置

            plt::xlim(-10, 10);
            plt::ylim(-10, 10);
            // 绘制表示航向的箭头
            plt::pause(0.01); // 暂停以更新图形
            // 模拟延时
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms 延时模拟实时性
        } else {
            std::cout << "已到达所有目标点，停止控制." << std::endl;
            break; // 如果已到达所有目标点，退出循环
        }
    }

    return 0;
}