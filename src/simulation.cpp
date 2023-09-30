#include "simulation.h"
#include "utils.h"


Simulation::Simulation() :
    m_sim_parameters(SimulationParams()),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(1),
    m_view_size(100),
    m_time(0.0),
    m_time_till_prediction(0.0),
    m_time_till_radar_measurement(0.0)
{
}

void Simulation::reset()
{
    // Reset Simulation
    m_time = 0.0;
    m_time_till_prediction = 0.0;
    m_time_till_radar_measurement = 0.0;

    m_is_running = true;
    m_is_paused = false;
    
    m_kalman_filter.reset();

    m_radar_sensor.reset();
    m_radar_sensor.setRadarNoiseStd(m_sim_parameters.radar_range_noise_std,
        m_sim_parameters.radar_theta_noise_std);

    m_car.reset(m_sim_parameters.car_initial_x, m_sim_parameters.car_initial_y,
        m_sim_parameters.car_initial_psi, m_sim_parameters.car_initial_velocity);
    
    for(auto& cmd : m_sim_parameters.car_commands)
    {
        m_car.addVehicleCommand(cmd.get());
    }

    // Plotting Variables
    m_radar_measurement_history.clear();
    m_vehicle_position_history.clear();
    m_filter_position_history.clear();

    // Stats Variables
    m_filter_error_x_position_history.clear();
    m_filter_error_y_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    std::cout << "Simulation: Reset" << std::endl;
}

void Simulation::update()
{
    if (m_is_running && !m_is_paused)
    {
        // Time Multiplier
        for (unsigned i = 0; i < m_time_multiplier; ++i)
        {
            // Check for End Time
            if(m_time >= m_sim_parameters.end_time)
            {
                m_is_running = false;
                std::cout << "Simulation: Reached End of Simulation Time ("
                          << m_time << ")" << std::endl;
                return;
            }

            // Update Motion
            m_car.update(m_time, m_sim_parameters.time_step);
            m_vehicle_position_history.push_back(
                Vector2(m_car.getVehicleState().x, m_car.getVehicleState().y));

            // Prediction Step
            if (m_time_till_prediction <= 0)
            {
                m_kalman_filter.predictionStep(m_sim_parameters.time_step);
                m_time_till_prediction += 1.0/m_sim_parameters.prediction_rate;
            }
            m_time_till_prediction -= m_sim_parameters.time_step;

            // Radar Measurement
            if (m_sim_parameters.radar_enabled)
            {
                if (m_time_till_radar_measurement <= 0)
                {
                    RadarMeasurement radar_measurement =
                        m_radar_sensor.generateRadarMeasurement(
                        m_car.getVehicleState().x, m_car.getVehicleState().y);
                    m_kalman_filter.handleRadarMeasurement(radar_measurement);
                    m_radar_measurement_history.push_back(radar_measurement);
                    m_time_till_radar_measurement += 1.0/m_sim_parameters.radar_update_rate;
                }
                m_time_till_radar_measurement -= m_sim_parameters.time_step;
            }

            // Save Filter History and Calculate Stats
            if (m_kalman_filter.isInitialised())
            {
                VehicleState vehicle_state = m_car.getVehicleState();
                VehicleState filter_state = m_kalman_filter.getVehicleState();
                m_filter_position_history.push_back(Vector2(filter_state.x, filter_state.y));
                m_filter_error_x_position_history.push_back(filter_state.x - vehicle_state.x);
                m_filter_error_y_position_history.push_back(filter_state.y - vehicle_state.y);
                m_filter_error_heading_history.push_back(
                    wrapAngle(filter_state.psi() - vehicle_state.psi()));
                m_filter_error_velocity_history.push_back(filter_state.V() - vehicle_state.V());
            }

            // Update Time
            m_time += m_sim_parameters.time_step;
        }
    }
}

void Simulation::render(Display& disp)
{
    std::vector<Vector2> marker_lines1 = {{0.5,  0.5}, {-0.5, -0.5}};
    std::vector<Vector2> marker_lines2 = {{0.5, -0.5}, {-0.5,  0.5}};

    disp.setView(m_view_size * disp.getScreenAspectRatio(), m_view_size,
        m_car.getVehicleState().x, m_car.getVehicleState().y);

    m_car.render(disp);

    disp.setDrawColour(0, 100, 0);
    disp.drawLines(m_vehicle_position_history);

    disp.setDrawColour(100, 0, 0);
    disp.drawLines(m_filter_position_history);

    if (m_kalman_filter.isInitialised())
    {
        VehicleState filter_state = m_kalman_filter.getVehicleState();
        Eigen::Matrix2d cov = m_kalman_filter.getVehicleStatePositionCovariance();

        double x = filter_state.x;
        double y = filter_state.y;
        double sigma_xx = cov(0, 0);
        double sigma_yy = cov(1, 1);
        double sigma_xy = cov(0, 1);

        std::vector<Vector2> marker_lines1_world = offsetPoints(marker_lines1, Vector2(x, y));
        std::vector<Vector2> marker_lines2_world = offsetPoints(marker_lines2, Vector2(x, y));
        disp.setDrawColour(255, 0, 0);
        disp.drawLines(marker_lines1_world);
        disp.drawLines(marker_lines2_world);

        std::vector<Vector2> cov_world = generateEllipse(x, y, sigma_xx, sigma_yy, sigma_xy);
        disp.setDrawColour(255, 0, 0);
        disp.drawLines(cov_world);

    }

    // Render Radar Measurements
    std::vector<std::vector<Vector2>> m_radar_marker =
        {{{0.5, 0.5},{-0.5, -0.5}}, {{0.5, -0.5},{-0.5, 0.5}}};
    disp.setDrawColour(0, 0, 255);
    for(const auto& meas : m_radar_measurement_history)
    {
        double x = meas.range*cos(meas.theta);
        double y = meas.range*sin(meas.theta);
        disp.drawLines(offsetPoints(m_radar_marker, Vector2(x, y)));
    }

    // Simulation Status / Parameters
    int stride = 20;
    int x_offset = 10;
    int y_offset = 30;
    std::string time_string = string_format("Time: %0.2f (x%d)",
        m_time, m_time_multiplier);
    std::string profile_string = string_format("Profile: %s",
        m_sim_parameters.profile_name.c_str());
    std::string radar_string = string_format("RADAR: %s (%0.1f Hz)",
        (m_sim_parameters.radar_enabled ? "ON" : "OFF"),
        m_sim_parameters.radar_update_rate);
    disp.drawText_MainFont(profile_string, Vector2(x_offset, y_offset+stride*-1),
        1.0, {255, 255, 255});
    disp.drawText_MainFont(time_string, Vector2(x_offset, y_offset+stride*0),
        1.0, {255, 255, 255});
    disp.drawText_MainFont(radar_string, Vector2(x_offset, y_offset+stride*1),
        1.0, {255, 255, 255});
    if (m_is_paused)
    {
        disp.drawText_MainFont("PAUSED",Vector2(x_offset,y_offset+stride*2),1.0,{255,0,0});
    }
    if (!m_is_running)
    {
        disp.drawText_MainFont("FINISHED",Vector2(x_offset,y_offset+stride*3), 1.0,{255,0,0});
    }

    // Vehicle State
    x_offset = 800;
    y_offset = 10;
    std::string velocity_string = string_format("    Velocity: %0.2f m/s",m_car.getVehicleState().V());
    std::string yaw_string =
        string_format("   Heading: %0.2f deg", m_car.getVehicleState().psi() * 180.0/M_PI);
    std::string xpos = string_format("X Position: %0.2f m", m_car.getVehicleState().x);
    std::string ypos = string_format("Y Position: %0.2f m", m_car.getVehicleState().y);
    disp.drawText_MainFont("Vehicle State",Vector2(x_offset-5,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(velocity_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(yaw_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont(xpos,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont(ypos,Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});

    // Filter state
    std::string kf_velocity_string =
        string_format("    Velocity: %0.2f m/s", m_kalman_filter.getVehicleState().V());
    std::string kf_yaw_string =
        string_format("   Heading: %0.2f deg", m_kalman_filter.getVehicleState().psi() * 180.0/M_PI);
    std::string kf_xpos =
        string_format("X Position: %0.2f m", m_kalman_filter.getVehicleState().x);
    std::string kf_ypos =
        string_format("Y Position: %0.2f m", m_kalman_filter.getVehicleState().y);
    disp.drawText_MainFont("Filter State", Vector2(x_offset, y_offset+stride*6), 1.0, {255, 255, 255});
    disp.drawText_MainFont(kf_velocity_string, Vector2(x_offset, y_offset+stride*7), 1.0, {255, 255, 255});
    disp.drawText_MainFont(kf_yaw_string, Vector2(x_offset, y_offset+stride*8), 1.0, {255, 255, 255});
    disp.drawText_MainFont(kf_xpos, Vector2(x_offset, y_offset+stride*9), 1.0, {255, 255, 255});
    disp.drawText_MainFont(kf_ypos, Vector2(x_offset, y_offset+stride*10), 1.0, {255, 255, 255});

    // Keyboard Input
    x_offset = 10;
    y_offset = 650;
    disp.drawText_MainFont("Reset Key: r", Vector2(x_offset, y_offset+stride*0),
        1.0, {255, 255, 255});
    disp.drawText_MainFont("Pause Key: [space bar]", Vector2(x_offset, y_offset+stride*1),
        1.0, {255, 255, 255});
    disp.drawText_MainFont("Speed Mult. (+/-) Key: [ / ] ", Vector2(x_offset, y_offset+stride*2),
        1.0, {255, 255, 255});
    disp.drawText_MainFont("Zoom (+/-) Key: ↑ / ↓", Vector2(x_offset, y_offset+stride*3),
        1.0, {255, 255, 255});
    disp.drawText_MainFont("Motion Profile Key: 1 - 9,0", Vector2(x_offset, y_offset+stride*4),
        1.0, {255, 255, 255});


    // Filter Error State
    x_offset = 750;
    y_offset = 650;
    std::string xpos_error_string = string_format("X Position RMSE: %0.2f m",
        calculateRMSE(m_filter_error_x_position_history));
    std::string ypos_error_string = string_format("Y Position RMSE: %0.2f m",
        calculateRMSE(m_filter_error_y_position_history));
    std::string heading_error_string = string_format("   Heading RMSE: %0.2f deg",
        180.0 / M_PI * calculateRMSE(m_filter_error_heading_history));
    std::string velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s",
        calculateRMSE(m_filter_error_velocity_history));
    disp.drawText_MainFont(xpos_error_string, Vector2(x_offset,y_offset+stride*0),
        1.0, {255, 255, 255});
    disp.drawText_MainFont(ypos_error_string, Vector2(x_offset,y_offset+stride*1),
        1.0, {255, 255, 255});
    disp.drawText_MainFont(heading_error_string, Vector2(x_offset,y_offset+stride*2),
        1.0, {255, 255, 255});
    disp.drawText_MainFont(velocity_error_string, Vector2(x_offset,y_offset+stride*3),
        1.0, {255, 255, 255});
}
   
void Simulation::reset(SimulationParams sim_params)
{
    m_sim_parameters = sim_params;
    reset();
}

void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier
              << ")" << std::endl;
}

void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")"
                  << std::endl;
    }
}

void Simulation::setTimeMultiplier(unsigned int multiplier)
{
    m_time_multiplier = static_cast<int>(multiplier);
}

void Simulation::increaseZoom()
{
    if (m_view_size > 25)
    {
        m_view_size -= 25;
    }
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}

void Simulation::decreaseZoom()
{
    if (m_view_size < 400)
    {
        m_view_size += 25;
    }
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}

void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")"
              << std::endl;
}

bool Simulation::isPaused()
{
    return m_is_paused;
}

bool Simulation::isRunning()
{
    return m_is_running;
}
