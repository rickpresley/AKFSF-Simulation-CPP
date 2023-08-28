#include "sensors.h"
#include "utils.h"

// Gyro Sensor
GyroSensor::GyroSensor() :
    m_rand_gen(std::mt19937()),
    m_noise_std(0.0),
    m_bias(0.0)
{
}

void GyroSensor::reset()
{
    m_rand_gen = std::mt19937();
}

void GyroSensor::setGyroNoiseStd(double std)
{
    m_noise_std = std;
}

void GyroSensor::setGyroBias(double bias)
{
    m_bias = bias;
}

GyroMeasurement GyroSensor::generateGyroMeasurement(double sensor_yaw_rate)
{
    GyroMeasurement meas;
    std::normal_distribution<double> gyro_dis(0.0, m_noise_std);
    meas.psi_dot = sensor_yaw_rate + m_bias + gyro_dis(m_rand_gen);
    return meas;
}

// Radar Sensor
RadarSensor::RadarSensor() :
    m_rand_gen(std::mt19937()),
    m_range_noise_std(0.0),
    m_theta_noise_std(0.0)
{
}

void RadarSensor::reset()
{
    m_rand_gen = std::mt19937();
}

void RadarSensor::setRadarNoiseStd(double range_std, double theta_std)
{
    m_range_noise_std = range_std;
    m_theta_noise_std = theta_std;
}

RadarMeasurement RadarSensor::generateRadarMeasurement(double sensor_x, double sensor_y)
{
    RadarMeasurement meas;
    std::normal_distribution<double> radar_theta_dis(0.0, m_theta_noise_std);
    std::normal_distribution<double> radar_range_dis(0.0, m_range_noise_std);
    double theta = wrapAngle(atan2(sensor_y, sensor_x));
    double range = std::sqrt(sensor_x*sensor_x + sensor_y*sensor_y);
    meas.theta = wrapAngle(theta + radar_theta_dis(m_rand_gen));
    meas.range = std::abs(range + radar_range_dis(m_rand_gen));
    return meas;
}
