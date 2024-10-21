#include <cmath>

class DiffDriveModel
{
private:
    double x, y, yaw, dt;
    double deg2rad = M_PI / 180;
public:
    DiffDriveModel(double x0, double y0, double yaw0);
    ~DiffDriveModel();
    void updateState(double v, double w);
    void getState(double &robot_x, double &robot_y, double &robot_yaw);
};

DiffDriveModel::DiffDriveModel(double x0, double y0, double yaw0)
{
    x = x0;
    y = y0;
    yaw = yaw0;
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