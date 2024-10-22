#ifndef LQR_CONTROLLER
#define LQR_CONTROLLER
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

class lqr_controller
{
private:
    double dt = 0.01;
public:
    MatrixXd A_, B_, Q_, R_, K_;
    lqr_controller();
    ~lqr_controller();
    // 使用LQR控制器计算控制输入
    void calAB(const std::vector<double>& v_theta);
    VectorXd control(const Eigen::VectorXd& state);
    void computeLQRGain();
    Eigen::MatrixXd solveAlgebraicRiccatiEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                                                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    MatrixXd solveRiccati(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R);
};

lqr_controller::lqr_controller()
{
    A_.resize(3, 3);
    A_.setZero();
    B_.resize(3, 2);
    B_.setZero();
    Q_.resize(3, 3);
    Q_ << 2, 0.0, 0.0,
         0.0, 2, 0.0, 
         0.0, 0.0, 2; 
    R_.resize(2, 2);
    R_ << 1.0, 0.0, 
         0.0, 1.0; 
}

lqr_controller::~lqr_controller()
{
}

void lqr_controller::calAB(const std::vector<double>& v_theta) {
    lqr_controller::A_ << 1.0, 0.0, -v_theta[0] * lqr_controller::dt * sin(v_theta[1]), 0.0, 1.0, v_theta[0] * lqr_controller::dt * cos(v_theta[1]), 0.0, 0.0, 1.0;
    lqr_controller::B_ << lqr_controller::dt * cos(v_theta[1]), 0.0, lqr_controller::dt * sin(v_theta[1]), 0.0, 0.0, lqr_controller::dt;
}
// 求解黎卡提方程
MatrixXd lqr_controller::solveRiccati(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R) {
    MatrixXd P = Q; // 初始猜测
    MatrixXd P_next;
    double tolerance = 1e-6;
    int max_iter = 1000;

    for (int i = 0; i < max_iter; ++i) {
        P_next = A.transpose() * P + P * A - P * B * R.inverse() * B.transpose() * P + Q;
        if ((P_next - P).norm() < tolerance)
            break;
        P = P_next;
    }

    return P;
}

// 计算LQR增益矩阵K
void lqr_controller::computeLQRGain() {
    // 计算代数黎卡特方程的解
    Eigen::MatrixXd P = solveAlgebraicRiccatiEquation(A_, B_, Q_, R_);
    K_ = R_.inverse() * B_.transpose() * P;
}

Eigen::MatrixXd lqr_controller::solveAlgebraicRiccatiEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                                                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    // 使用数值方法求解代数黎卡特方程
    Eigen::MatrixXd P = Q; // 初始值
    for (int i = 0; i < 1000; ++i) { // 迭代求解
        Eigen::MatrixXd P_next = Q + A.transpose() * P * A - 
                                    A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if ((P_next - P).norm() < 1e-6) {
            break;
        }
        P = P_next;
    }
    return P;
}

VectorXd lqr_controller::control(const Eigen::VectorXd& state) {
    return -K_ * state;
}

#endif