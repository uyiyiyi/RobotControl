#include "mpc_controller.h"
#include "draw.h"


int main() {
    int num_points = 100; // 生成的轨迹点数量
    // 获取圆轨迹
    std::vector<std::vector<double>> trajectory = generateCircleTrajectory(num_points);

    // 初始化MPC控制器
    mpc_controller mpc;

    // // 定义差分驱动机器人模型的初始状态
    VectorXd init_state(3);
    init_state << 0.0, 0.0, 0.0; // 初始状态 x=0, y=0, theta=0
    VectorXd current_state, X_ref;
    double robot_x, robot_y, robot_yaw;
    std::vector<double> ctrl = {2.0, 0.0}; // 线速度和角速度
    std::vector<double> v_theta = {0.0, 0.0};
    
    DiffDriveModel robot(init_state); // 使用已有的差分驱动模型

    // 控制循环
    while (true) {
        // 获取机器人当前状态
        robot.getState(robot_x, robot_y, robot_yaw); // 假设这个函数存在
        current_state << robot_x, robot_y, robot_yaw;

        mpc.calABQRHf(v_theta, current_state, X_ref);
        // 求解MPC问题，获取控制输入
        VectorXd control_input = mpc.solveQP();

        // 使用控制输入更新机器人状态
        robot.updateState(control_input(0), control_input(1)); // 假设此函数更新机器人状态

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
    }

    return 0;
}