// -------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset,
    const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const LidarMeasurement& meas : dataset)
    {
        handleLidarMeasurement(meas, map);
    }
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ------------------------------------------------------------------ //

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built-
                                                              // in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {
            Vector2d z;
            z << meas.range, meas.theta;

            // The map matched beacon positions can be accessed using: map_beacon.x AND map_beacon.y
            double delta_x = map_beacon.x - state[0];
            double delta_y = map_beacon.y - state[1];
            double r_hat = sqrt(delta_x*delta_x + delta_y*delta_y);
            double theta_hat = wrapAngle(atan2(delta_y, delta_x) - state[2]);
            Vector2d z_hat;
            z_hat << r_hat, theta_hat;

            // Calculate the measurement innovation
            Vector2d v = z - z_hat;
            v[1] = wrapAngle(v[1]);
            Matrix2d R;
            R << LIDAR_RANGE_STD*LIDAR_RANGE_STD, 0.0,
                 0.0, LIDAR_THETA_STD*LIDAR_THETA_STD;

            MatrixXd H(2, 4);
            H <<        -delta_x/r_hat,         -delta_y/r_hat,  0.0, 0.0,
                 delta_y/(r_hat*r_hat), -delta_x/(r_hat*r_hat), -1.0, 0.0;

            MatrixXd S = H * cov * H.transpose() + R;
            MatrixXd K = cov * H.transpose() * S.inverse();

            state = state + K * v;
            cov = (Matrix4d::Identity() - K * H) * cov;
        }

        // ------------------------------------------------------------------ //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // HINT: Assume the state vector has the form [PX, PY, PSI, V].
        // HINT: Use the Gyroscope measurement as an input into the prediction step.
        // HINT: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // ------------------------------------------------------------------ //

        double px = state[0];
        double py = state[1];
        double psi = state[2];
        double V = state[3];

        double px_new = px + dt*V*cos(psi);
        double py_new = py + dt*V*sin(psi);
        double psi_new = wrapAngle(psi + dt * gyro.psi_dot);
        double V_new = V;
        state << px_new,
                 py_new,
                 psi_new,
                 V_new;

        Matrix4d F;
        F << 1.0, 0.0, -dt*V*sin(psi), dt*cos(psi),
             0.0, 1.0,  dt*V*cos(psi), dt*sin(psi),
             0.0, 0.0,            1.0,         0.0,
             0.0, 0.0,            0.0,         1.0;

        Matrix4d Q;
        Q << 0.0, 0.0,                     0.0,                       0.0,
             0.0, 0.0,                     0.0,                       0.0,
             0.0, 0.0, dt*dt*GYRO_STD*GYRO_STD,                       0.0,
             0.0, 0.0,                     0.0, dt*dt*ACCEL_STD*ACCEL_STD;

        cov = F * cov * F.transpose() + Q;

        // ------------------------------------------------------------------ //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the EKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2, 4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H << 1, 0, 0, 0,
             0, 1, 0, 0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        cov = (Matrix4d::Identity() - K*H) * cov;

        setState(state);
        setCovariance(cov);
    }
    else
    {
        if(! (m_init_x && m_init_y) )
        {
            m_init_x = meas.x;
            m_init_y = meas.y;
        }
        else if( ! m_init_hdg )
        {
            double delta_x = meas.x - m_init_x.value();
            double delta_y = meas.y - m_init_y.value();
            if ((delta_x*delta_x + delta_y*delta_y) > 3*GPS_POS_STD)
            {
                m_init_hdg = wrapAngle(atan2(delta_y, delta_x));
                m_init_x = meas.x;
                m_init_y = meas.y;
            }
        }

        if( ! m_init_vel )
        {
            m_init_vel = 0.0;
        }

        if( m_init_x && m_init_y && m_init_hdg && m_init_vel )
        {
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();

            state(0) = m_init_x.value();
            state(1) = m_init_y.value();
            state(2) = m_init_hdg.value();
            state(3) = m_init_vel.value();

            cov(0, 0) = GPS_POS_STD*GPS_POS_STD;
            cov(1, 1) = GPS_POS_STD*GPS_POS_STD;
            cov(2, 2) = LIDAR_THETA_STD*LIDAR_THETA_STD;
            cov(3, 3) = INIT_VEL_STD*INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        }
    } 
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0)
    {
        pos_cov << cov(0,0), cov(0,1),
                   cov(1,0), cov(1,1);
    }
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0], state[1], state[2], state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt)
{
}
