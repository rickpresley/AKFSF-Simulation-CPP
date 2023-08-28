#pragma once

#include <memory>
#include <vector>

#include "kalmanfilter.h"
#include "display.h"
#include "car.h"
#include "sensors.h"


struct SimulationParams
{
    std::string profile_name;
    double time_step;
    double end_time;

    bool gyro_enabled;
    double gyro_update_rate;
    double gyro_noise_std;
    double gyro_bias;

    bool radar_enabled;
    double radar_update_rate;
    double radar_range_noise_std;
    double radar_theta_noise_std;

    double car_initial_x;
    double car_initial_y;
    double car_initial_psi;
    double car_initial_velocity;

    std::vector<std::shared_ptr<MotionCommandBase>> car_commands;

    SimulationParams() : profile_name(""), time_step(0.1), end_time(120),
        gyro_enabled(true), gyro_update_rate(10.0),gyro_noise_std(0.001), gyro_bias(0.0),
        radar_enabled(true), radar_update_rate(2.0), radar_range_noise_std(1.0), radar_theta_noise_std(0.01),
        car_initial_x(0.0), car_initial_y(0.0), car_initial_psi(0.0), car_initial_velocity(5.0)
    {
    }
};


class Simulation
{
public:

    Simulation();
    void reset();
    void reset(SimulationParams sim_params);
    void update();
    void render(Display& disp);
    void increaseTimeMultiplier();
    void decreaseTimeMultiplier();
    void setTimeMultiplier(unsigned int multiplier);
    void increaseZoom();
    void decreaseZoom();
    void togglePauseSimulation();
    bool isPaused();
    bool isRunning();

private:

    SimulationParams m_sim_parameters;
    KalmanFilter m_kalman_filter;
    Car m_car;
    GyroSensor m_gyro_sensor;
    RadarSensor m_radar_sensor;

    bool m_is_paused;
    bool m_is_running;
    int  m_time_multiplier;
    double m_view_size;

    double m_time;
    double m_time_till_gyro_measurement;
    double m_time_till_radar_measurement;

    std::vector<RadarMeasurement> m_radar_measurement_history;

    std::vector<Vector2> m_vehicle_position_history;
    std::vector<Vector2> m_filter_position_history;

    std::vector<double> m_filter_error_x_position_history;
    std::vector<double> m_filter_error_y_position_history;
    std::vector<double> m_filter_error_heading_history;
    std::vector<double> m_filter_error_velocity_history;

};
