// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
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

// ----------------------------------------------------------------------- //
// USEFUL HELPER FUNCTIONS
VectorXd normaliseState(VectorXd state)
{
    state(2) = wrapAngle(state(2));
    return state;
}

VectorXd normaliseLidarMeasurement(VectorXd meas)
{
    meas(1) = wrapAngle(meas(1));
    return meas;
}

std::vector<VectorXd> generateSigmaPoints(VectorXd state, MatrixXd cov)
{
    std::vector<VectorXd> sigmaPoints;

    // ----------------------------------------------------------------------- //

    int n = state.size();
    double k = 3.0 - n;
    MatrixXd sqrtCov = cov.llt().matrixL();
    sigmaPoints.push_back(state);
    for(int i=0; i<n; i++)
    {
        sigmaPoints.push_back(state + sqrt(k + n) * sqrtCov.col(i));
        sigmaPoints.push_back(state - sqrt(k + n) * sqrtCov.col(i));
    }

    // ----------------------------------------------------------------------- //

    return sigmaPoints;
}

std::vector<double> generateSigmaWeights(unsigned int n)
{
    std::vector<double> weights;

    // ----------------------------------------------------------------------- //

    double k = 3.0 - n;
    weights.push_back(k / (n + k));
    for(int i=0; i<2*n; i++)
    {
        weights.push_back(1.0/(2.0*(n+k)));
    }

    // ----------------------------------------------------------------------- //

    return weights;
}

VectorXd lidarMeasurementModel(VectorXd aug_state, double beaconX, double beaconY)
{
    VectorXd z_hat = VectorXd::Zero(2);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE

    double p_x = aug_state[0];
    double p_y = aug_state[1];
    double psi = aug_state[2];
    double range_noise = aug_state[4];
    double theta_noise = aug_state[5];

    double x_diff = beaconX - p_x;
    double y_diff = beaconY - p_y;
    double r_hat = sqrt(x_diff*x_diff + y_diff*y_diff) + range_noise;
    double theta_hat = atan2(y_diff, x_diff) - psi + theta_noise;

    z_hat << r_hat,
             theta_hat;

    // ---------------------------------------------------------------------- //

    return z_hat;
}

VectorXd vehicleProcessModel(VectorXd aug_state, double psi_dot, double dt)
{
    VectorXd new_state = VectorXd::Zero(4);

    // ---------------------------------------------------------------------- //

    double p_x = aug_state[0];
    double p_y = aug_state[1];
    double psi = aug_state[2];
    double V = aug_state[3];
    double w_psi_dot = aug_state[4];
    double w_a = aug_state[5];

    new_state << p_x + dt*V*cos(psi),
                 p_y + dt*V*sin(psi),
                 psi + dt*(psi_dot + w_psi_dot),
                 V + dt*w_a;

    // ---------------------------------------------------------------------- //

    return new_state;
}
// -------------------------------------------------------------------------- //

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
        // to always keep angle values within correct range.
        // HINT: Do not normalise during sigma point calculation!
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ------------------------------------------------------------------ //
        // ENTER YOUR CODE HERE

        // Match Beacon with built in Data Association Id
        BeaconData map_beacon = map.getBeaconWithId(meas.id);
        if (meas.id != -1 && map_beacon.id != -1) // Check that we have a valid beacon match
        {
            VectorXd z = Vector2d::Zero();
            z << meas.range,
                 meas.theta;

            int size = state.size();
            int aug_size = size+2;
            VectorXd state_aug = VectorXd::Zero(aug_size);
            MatrixXd cov_aug = MatrixXd::Zero(aug_size, aug_size);
            state_aug.head(size) = state;
            cov_aug.topLeftCorner(size, size) = cov;
            cov_aug(size, size) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
            cov_aug(size+1, size+1) = LIDAR_THETA_STD*LIDAR_THETA_STD;

            // Generate augmented sigma points and weights
            std::vector<VectorXd> sigma_points = generateSigmaPoints(state_aug, cov_aug);
            std::vector<double> sigma_weights = generateSigmaWeights(aug_size);

            // Run sigma points through the measurement model
            std::vector<VectorXd> z_sig;
            for (const auto& point : sigma_points)
            {
                z_sig.push_back(lidarMeasurementModel(point, map_beacon.x, map_beacon.y));
            }

            // Calculate the measurement mean
            VectorXd z_mean = VectorXd::Zero(2);
            for (int i=0; i<z_sig.size(); i++)
            {
                z_mean += sigma_weights[i] * z_sig[i];
            }

            // Calculate the innovation covariance
            MatrixXd Sk = MatrixXd::Zero(2, 2);
            for(int i=0; i<z_sig.size(); i++)
            {
                VectorXd diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Sk += sigma_weights[i] * diff * diff.transpose();
            }

            // Calculate the cross covariance
            MatrixXd Pxz = MatrixXd::Zero(size, 2);
            for (int i=0; i<sigma_points.size(); i++)
            {
                VectorXd x_diff = normaliseState(sigma_points[i].head(size) - state);
                VectorXd z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Pxz += sigma_weights[i] * x_diff * z_diff.transpose();
            }

            MatrixXd K = Pxz * Sk.inverse();
            VectorXd y = normaliseLidarMeasurement(z - z_mean);
            state = state + K*y;
            cov = cov - K * Sk * K.transpose();
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
        // HINT: Use the normaliseState() function to always keep angle values within correct range.
        // HINT: Do NOT normalise during sigma point calculation!
        // ----------------------------------------------------------------------- //

        int size = state.size();
        int aug_size = size+2;

        VectorXd state_aug = VectorXd::Zero(aug_size);
        state_aug.head(size) = state;

        MatrixXd cov_aug = MatrixXd::Zero(aug_size, aug_size);
        cov_aug.topLeftCorner(size, size) = cov;
        cov_aug(size, size) = GYRO_STD*GYRO_STD;
        cov_aug(size+1, size+1) = ACCEL_STD*ACCEL_STD;

        std::vector<VectorXd> sigma_points = generateSigmaPoints(state_aug, cov_aug);
        std::vector<double> sigma_weights = generateSigmaWeights(aug_size);

        std::vector<VectorXd> sigma_points_predict;
        for (const auto& sigma_point : sigma_points)
        {
            sigma_points_predict.push_back(vehicleProcessModel(sigma_point, gyro.psi_dot, dt));
        }

        state = VectorXd::Zero(size);
        for(int i=0; i<sigma_points_predict.size(); i++)
        {
            state += sigma_weights[i]*sigma_points_predict[i];
        }
        state = normaliseState(state);

        cov = MatrixXd::Zero(size, size);
        for(int i=0; i<sigma_points_predict.size(); i++)
        {
            VectorXd diff = normaliseState(sigma_points_predict[i] - state);
            cov += sigma_weights[i] * diff * diff.transpose();
        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the UKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,
             meas.y;
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        cov = (MatrixXd::Identity(4,4) - K*H) * cov;

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // You may modify this initialisation routine if you can think of a more
        // robust and accuracy way of initialising the filter.
        // ------------------------------------------------------------------ //
        // YOU ARE FREE TO MODIFY THE FOLLOWING CODE HERE

        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);

        // ------------------------------------------------------------------ //
    }             
}

void KalmanFilter::handleLidarMeasurements(
    const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset)
    {
        handleLidarMeasurement(meas, map);
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
