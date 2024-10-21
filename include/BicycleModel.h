#include <cmath>

class BicycleModel
{
private:
    double x, y, yaw, dt;
    double deg2rad = M_PI / 180;
public:
    BicycleModel(double x0, double y0, double yaw0);
    ~BicycleModel();
    void updateState(double v, double w);
    void getState(double &robot_x, double &robot_y, double &robot_yaw);
};

BicycleModel::BicycleModel(double x0, double y0, double yaw0)
{
    x = x0;
    y = y0;
    yaw = yaw0;
}

BicycleModel::~BicycleModel()
{
}

void BicycleModel::updateState(double v, double w)
{
    yaw += w * dt;
    x += v * cos(yaw);
    y += v * sin(yaw);
}

void BicycleModel::getState(double &robot_x, double &robot_y, double &robot_yaw)
{
    robot_x = x;
    robot_y = y;
    robot_yaw = yaw;
}