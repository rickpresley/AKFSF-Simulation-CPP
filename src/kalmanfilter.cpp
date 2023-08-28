// ------------------------------------------------------------------------------- //
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
constexpr double INIT_POS_STD = 0.0;
constexpr double INIT_VEL_STD = 10.0;
constexpr double RADAR_RANGE_STD = 3.0;
constexpr double RADAR_THETA_STD = 0.01;
// -------------------------------------------------- //

void KalmanFilter::handleRadarMeasurement(RadarMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Radar Measurements in
        // the section below.
        // Hint: You can use the constants: RADAR_RANGE_STD, RADAR_THETA_STD
        // ------------------------------------------------------------------ //
        double x = state[0],
               y = state[1];
        double r_sq = x*x + y*y;
        double r = sqrt(r_sq);
        double azimuth = atan2(y, x);

        VectorXd z = Vector2d::Zero();
        z << meas.range, meas.theta;

        MatrixXd H(2, 4);
        H <<     x/r,    y/r, 0.0, 0.0,
             -y/r_sq, x/r_sq, 0.0, 0.0;

        VectorXd h = Vector2d::Zero();
        h << r, azimuth;

        MatrixXd R = Matrix2d::Zero();
        R(0, 0) = RADAR_RANGE_STD*RADAR_RANGE_STD;
        R(1, 1) = RADAR_THETA_STD*RADAR_THETA_STD;

        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        // A posteriori state and covariance updates using measurement
        state = state + K*(z - h);
        cov = (Matrix4d::Identity() - K*H) * cov;

        // ------------------------------------------------------------------ //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        VectorXd state = Vector4d::Zero();
        state(0) = meas.range*cos(meas.theta);
        state(1) = meas.range*sin(meas.theta);

        MatrixXd cov = Matrix4d::Zero();
        cov(0,0) = INIT_POS_STD*INIT_POS_STD;
        cov(1,1) = INIT_POS_STD*INIT_POS_STD;
        cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt) { predictionStep(dt);}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // Hint: You can use the constants: ACCEL_STD
        // ----------------------------------------------------------------------- //
        
        MatrixXd F = Matrix4d();
        F << 1, 0, dt,  0,
             0, 1,  0, dt,
             0, 0,  1,  0,
             0, 0,  0,  1;

        MatrixXd Q = Matrix2d::Zero();
        Q(0,0) = (ACCEL_STD*ACCEL_STD);
        Q(1,1) = (ACCEL_STD*ACCEL_STD);

        MatrixXd L = MatrixXd(4,2);
        L << (0.5*dt*dt),           0,
                       0, (0.5*dt*dt),
                      dt,           0,
                       0,          dt;

        state = F*state;
        cov = F*cov*F.transpose() + L*Q*L.transpose();

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}
