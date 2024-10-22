#ifndef DIFFDRIVEMODEL
#define DIFFDRIVEMODEL
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

class DiffDriveModel
{
private:
    double x, y, yaw, dt;
    double deg2rad = M_PI / 180;
public:
    DiffDriveModel(VectorXd init_state);
    ~DiffDriveModel();
    void updateState(double v, double w);
    void getState(double &robot_x, double &robot_y, double &robot_yaw);
};

DiffDriveModel::DiffDriveModel(VectorXd init_state)
{
    x = init_state(0);
    y = init_state(1);
    yaw = init_state(2);
    dt = 0.01;
}

DiffDriveModel::~DiffDriveModel()
{
}

void DiffDriveModel::updateState(double v, double w)
{
    yaw += w * dt;
    x += v * dt * cos(yaw);
    y += v * dt * sin(yaw);
}

void DiffDriveModel::getState(double &robot_x, double &robot_y, double &robot_yaw)
{
    robot_x = x;
    robot_y = y;
    robot_yaw = yaw;
}
#endif // DIFFDRIVEMODEL