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
using namespace std;

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
    double dt; // 时间步长
    int N;     // 预测步长
    int state_dim;
    int control_dim;
public:
    MatrixXd A, B, A_big, B_big, Q, R, Q_big, R_big, H, f;
    mpc_controller(double T, int n, int state, int control);
    ~mpc_controller();
    void calABQRHf(const std::vector<double>& v_theta, const Eigen::VectorXd& X_0, const Eigen::VectorXd& X_ref);
    VectorXd solveQP();
};

mpc_controller::mpc_controller(double T, int n, int state, int control)
{
    dt = T;
    N = n;
    state_dim = state;
    control_dim = control;
    A = MatrixXd::Zero(state_dim, state_dim);
    B = MatrixXd::Zero(state_dim, control_dim);
    A_big = MatrixXd::Zero(state_dim * N, state_dim);
    B_big = MatrixXd::Zero(state_dim * N, control_dim * N);
    Q = MatrixXd::Identity(state_dim, state_dim);
    R = MatrixXd::Identity(control_dim, control_dim);
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
    cout << "A is: " << A << endl;
    cout << "B is: " << B << endl;
    for (size_t i = 0; i < N; i++)
    {
        A_big.block(i * state_dim, 0, state_dim, state_dim) = MatrixPower(A, i+1);
        Q_big.block(i * state_dim, i * state_dim, state_dim, state_dim) = 10 * Q;
        R_big.block(i * control_dim, i * control_dim, control_dim, control_dim) = 0.1 * R;
    }
    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j <= i; j++)
        {
            B_big.block(i * state_dim, j * control_dim, state_dim, control_dim) = MatrixPower(A, i - j) * B;
        }
    }
    cout << "A_big is: " << A_big << endl;
    cout << "Q_big is: " << Q_big << endl;
    cout << "R_big is: " << R_big << endl;
    cout << "B_big is: " << B_big << endl;
    // 计算 H 和 f
    cout << "B_big shape: " << B_big.rows() << " * " << B_big.cols() << endl;
    cout << "Q_big shape: " << Q_big.rows() << " * " << Q_big.cols() << endl;
    cout << "R_big shape: " << R_big.rows() << " * " << R_big.cols() << endl;
    cout << "A_big shape: " << A_big.rows() << " * " << A_big.cols() << endl;
    cout << "X_0 shape: " << X_0.rows() << " * " << X_0.cols() << endl;
    cout << "X_ref shape: " << X_ref.rows() << " * " << X_ref.cols() << endl;
    cout << "X_ref is: " << X_ref << endl;
    H = B_big.transpose() * Q_big * B_big + R_big;
    cout << "H shape: " << H.rows() << " * " << H.cols() << endl;
    cout << "H is: " << endl << H << endl;
    f = B_big.transpose() * Q_big * (A_big * X_0 - X_ref);
    cout << "f shape: " << f.rows() << " * " << f.cols() << endl;
    cout << "f is: " << endl << f << endl;
}

VectorXd mpc_controller::solveQP() 
{
    // instantiate the solver
    OsqpEigen::Solver solver;
    int num_constraints = 0;

    // settings
    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(control_dim * N);   //变量数
    solver.data()->setNumberOfConstraints(num_constraints); //约束数

    cout << "H is: " << endl << H << endl;
    SparseMatrix<double> Hessian_sparse = H.sparseView();
    VectorXd f_ = f;
    cout << "Hessian_sparse is: " << endl << Hessian_sparse << endl;
    cout << "f_ is: " << endl << f_ << endl;
    
    solver.data()->setHessianMatrix(Hessian_sparse);
    solver.data()->setGradient(f_);

    if (!solver.initSolver())
    {
        throw std::runtime_error("OSQP initialization failed!");
    }

    if(!solver.solve())
    {
        throw std::runtime_error("OSQP solve failed!");
    }
    
    VectorXd optimal_control = solver.getSolution();

    cout << "_______________________________________________________________________________" << endl;
    cout << "_______________________________________________________________________________" << endl;
    cout << "optimal_control: " << optimal_control << endl;
    cout << "_______________________________________________________________________________" << endl;
    cout << "_______________________________________________________________________________" << endl;

    return optimal_control;
}
#endif // !MPC_CONTROLLER