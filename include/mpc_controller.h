#ifndef MPC_CONTROLLER
#define MPC_CONTROLLER
#include <iostream>
#include <thread>
#include <cmath>
#include <Eigen/Dense>
// osqp-eigen库
#include "OsqpEigen/OsqpEigen.h"
#include "DiffDriveModel.h"

using namespace Eigen;

MatrixXd MatrixPower(MatrixXd Mat, int n)
{
    MatrixXd A_n = Eigen::MatrixXd::Identity(Mat.rows(), Mat.cols()); // 初始化为单位矩阵
    for (size_t i = 0; i < n; i++)
    {
        A_n = A_n * Mat;
    }
    return A_n;
}

class mpc_controller
{
private:
    /* data */
    double dt = 0.1; // 时间步长
    int N = 10;     // 预测步长
    MatrixXd A, B, A_big, B_big, Q, R, Q_big, R_big, H, f;
    int state_dim = 3;
    int control_dim = 2;
public:
    mpc_controller(/* args */);
    ~mpc_controller();
    void calABQRHf(const std::vector<double>& v_theta, const Eigen::VectorXd& X_0, const Eigen::VectorXd& X_ref);
    VectorXd solveQP();
};

mpc_controller::mpc_controller(/* args */)
{
    A = MatrixXd::Zero(state_dim, state_dim);
    B = MatrixXd::Zero(state_dim, control_dim);
    A_big = MatrixXd::Zero(state_dim * N, state_dim);
    B_big = MatrixXd::Zero(state_dim * N, control_dim * N);
    Q = MatrixXd::Zero(state_dim, state_dim);
    R = MatrixXd::Zero(control_dim, control_dim);
    Q_big = MatrixXd::Zero(state_dim * N, state_dim * N);
    R_big = MatrixXd::Zero(control_dim * N, control_dim * N);
    H = MatrixXd::Zero(control_dim * N, control_dim * N);
    f = MatrixXd::Zero(control_dim * N, 1);
}

mpc_controller::~mpc_controller()
{
}

// X_0 为初始状态 R^(3, 1) (x, y, theta), X_ref为N个预瞄点的状态 R^(3 * N, 1)
void mpc_controller::calABQRHf(const std::vector<double>& v_theta, const Eigen::VectorXd& X_0, const Eigen::VectorXd& X_ref)
{
    // 计算 A, B, Q, R
    A << 1, 0, -dt * v_theta[0] * sin(v_theta[1]), 0, 1, dt * v_theta[0] * cos(v_theta[1]), 0, 0, 1;
    B << dt * cos(v_theta[1]), 0, dt * sin(v_theta[1]), 0, 0, dt;
    for (size_t i = 0; i < N; i++)
    {
        A_big.block(i * state_dim, 0, state_dim, state_dim) = MatrixPower(A, i+1);
        Q_big.block(i * state_dim, i * state_dim, state_dim, state_dim) = Q;
        R_big.block(i * control_dim, i * control_dim, control_dim, control_dim) = R;
    }
    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j <= i; j++)
        {
            B_big.block(i * state_dim, j * control_dim, state_dim, control_dim) = MatrixPower(A, i - j) * B;
        }
    }
    // 计算 H 和 f
    Eigen::MatrixXd H = B_big.transpose() * Q_big * Q_big + R_big;
    Eigen::VectorXd f = B_big.transpose() * Q_big * (A_big * X_0 - X_ref);
}

VectorXd mpc_controller::solveQP() 
{
    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(control_dim * N);   //变量数
    solver.data()->setNumberOfConstraints(control_dim * N); //约束数

    SparseMatrix<double> Hessian_sparse = H.sparseView();
    VectorXd f_ = f;
    
    solver.data()->setHessianMatrix(Hessian_sparse);
    solver.data()->setGradient(f_);

    if (!solver.initSolver())
    {
        throw std::runtime_error("OSQP initialization failed!");
    }

    VectorXd optimal_control = solver.getSolution();

    return optimal_control;
}
#endif // !MPC_CONTROLLER