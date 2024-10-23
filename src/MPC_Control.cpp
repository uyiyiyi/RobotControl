#include "mpc_controller.h"
#include "draw.h"



int main() {
    // 初始化MPC控制器
    mpc_controller mpc;

    // // 定义差分驱动机器人模型的初始状态
    VectorXd init_state(3);
    init_state << 0.0, 0.0, 0.0; // 初始状态 x=0, y=0, theta=0
    VectorXd current_state;
    double robot_x, robot_y, robot_yaw;
    std::vector<double> ctrl = {2.0, 0.0}; // 线速度和角速度
    std::vector<double> v_theta = {0.0, 0.0};
    
    DiffDriveModel robot(init_state); // 使用已有的差分驱动模型

    // 控制循环
    while (true) {
        // 获取机器人当前状态
        robot.getState(robot_x, robot_y, robot_yaw); // 假设这个函数存在
        current_state << robot_x, robot_y, robot_yaw;

        mpc.calABQRHf(v_theta, current_state, VectorXd& X_ref);
        // 求解MPC问题，获取控制输入
        VectorXd control_input = mpc.solve(current_state, x_ref);

        // 使用控制输入更新机器人状态
        robot.updateState(control_input(0), control_input(1)); // 假设此函数更新机器人状态

        // 输出当前状态
        std::cout << "Current State: " << current_state.transpose() << std::endl;

        // 模拟延时
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 延时模拟实时性
    }

    return 0;
}