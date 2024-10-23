#ifndef DRAW
#define DRAW
#include "matplotlibcpp.h"

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

#endif