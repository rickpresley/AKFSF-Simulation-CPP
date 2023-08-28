#pragma once

#include <random>
#include <vector>

struct GyroMeasurement
{
    double psi_dot;
};

struct RadarMeasurement
{
    double range, theta;
};

class GyroSensor
{
public:

    GyroSensor();
    void reset();
    void setGyroNoiseStd(double std);
    void setGyroBias(double bias);
    GyroMeasurement generateGyroMeasurement(double sensor_yaw_rate);

private:

    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_bias;
};

class RadarSensor
{
public:

    RadarSensor();
    void reset();
    void setRadarNoiseStd(double range_std, double theta_std);
    RadarMeasurement generateRadarMeasurement(double sensor_x, double sensor_y);

private:

    std::mt19937 m_rand_gen;
    double m_range_noise_std;
    double m_theta_noise_std;
};
