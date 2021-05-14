#ifndef ETQ_HPP
#define ETQ_HPP
#include <vector>
typedef std::vector<std::vector<double>> arucoPose;
struct Quaternion
{
    double w, x, y, z;
};
struct EulerAngles {
    double roll, pitch, yaw;
};
Quaternion ToQuaternion(double yaw, double pitch, double roll); // yaw (Z), pitch (Y), roll (X)
EulerAngles ToEulerAngles(Quaternion q);
#endif