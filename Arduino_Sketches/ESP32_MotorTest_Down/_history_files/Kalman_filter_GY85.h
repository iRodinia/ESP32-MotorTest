#ifndef GY85_KALMAN_FILTER_H
#define GY85_KALMAN_FILTER_H

#include <Arduino.h>
#include <math.h>

#define PI 3.14159265358979f

class GY85_KalmanFilter {
public:
    GY85_KalmanFilter();
    void init();
    void update(float accel_x, float accel_y, float accel_z,  // m/s^2
                float gyro_x, float gyro_y, float gyro_z,    // rad/s
                float mag_x, float mag_y, float mag_z,       // uT
                float dt);                                    // s
    
    void getVelocity(float &vx, float &vy, float &vz);      // m/s
    void getAcceleration(float &ax, float &ay, float &az);  // m/s^2
    void getAttitude(float &roll, float &pitch, float &yaw); // rad
    void getAngularVelocity(float &wx, float &wy, float &wz); // rad/s
    
    void setProcessNoise(float q_accel, float q_gyro, float q_attitude);
    void setMeasurementNoise(float r_accel, float r_mag, float r_gyro);
    void reset();
    bool getInitStatus();

private:
    // State vector: [vx, vy, vz, ax, ay, az, q0, q1, q2, q3, wx, wy, wz]
    // World frame: x=East, y=North, z=Up (ENU)
    // Body frame: x=Forward, y=Left, z=Up (FLU)
    // Sensor mounting: x=Forward, y=Right, z=Down (FRD)
    // vx, vy, vz: velocity in world frame (m/s)
    // ax, ay, az: acceleration in world frame (m/s^2)
    // q0-q3: quaternion (attitude, world to body frame)
    // wx, wy, wz: angular velocity in body frame FLU (rad/s)
    float state[13];
    float P[13][13];  // covariance matrix
    
    float Q_accel;
    float Q_gyro;
    float Q_attitude;
    float R_accel;
    float R_mag;
    float R_gyro;
    
    bool initialized;
    float mag_ref[3];  // reference magnetic field in world frame
    
    // Helper functions
    void predictState(float dt, float gyro[3], float accel_body[3]);
    void updateWithAccel(float accel_body[3]);
    void updateWithMag(float mag[3]);
    void updateWithGyro(float gyro[3]);
    
    void quaternionNormalize(float q[4]);
    void quaternionToEuler(float q[4], float &roll, float &pitch, float &yaw);
    void quaternionMultiply(float q1[4], float q2[4], float result[4]);
    void rotateVectorByQuaternion(float q[4], float v[3], float result[3]);
    void rotateVectorByQuaternionInv(float q[4], float v[3], float result[3]);
    void compensateGravity(float accel_body[3], float q[4], float accel_world[3]);
    
    void matrixMultiply(float A[13][13], float B[13][13], float result[13][13]);
    void matrixAddition(float A[13][13], float B[13][13], float result[13][13]);
};

GY85_KalmanFilter::GY85_KalmanFilter() {
    Q_accel = 0.1f;
    Q_gyro = 0.01f;
    Q_attitude = 0.001f;
    
    R_accel = 0.5f;
    R_mag = 0.5f;
    R_gyro = 0.1f;
    
    initialized = false;
    
    // Reference magnetic field for this location
    // Inclination angle: -3°16' (negative means below horizontal)
    // Declination angle: West (negative)
    // Convert to radians: -3.2667° = -0.05701 rad
    float inclination = -3.2667f * PI / 180.0f;  // -3°16' converted to degrees
    
    // Magnetic field components in world frame (NED: North-East-Down)
    // North: horizontal component (cos of inclination)
    // East: 0 (assuming no magnetic declination correction needed here)
    // Down: vertical component (sin of inclination, negative means up)
    mag_ref[0] = cos(inclination);   // North component (~0.9983)
    mag_ref[1] = 0.0f;               // East component
    mag_ref[2] = sin(inclination);   // Down component (~-0.0568, negative means upward)
    
    // Normalize the reference magnetic field
    float mag_norm = sqrt(mag_ref[0]*mag_ref[0] + mag_ref[1]*mag_ref[1] + mag_ref[2]*mag_ref[2]);
    if(mag_norm > 0.0001f) {
        for(int i = 0; i < 3; i++) {
            mag_ref[i] /= mag_norm;
        }
    }
}

bool GY85_KalmanFilter::getInitStatus() {
    return initialized;
}

void GY85_KalmanFilter::init() {
    // Initialize state vector
    for(int i = 0; i < 13; i++) {
        state[i] = 0.0f;
    }
    state[6] = 1.0f;  // q0 = 1 (identity quaternion)
    
    // Initialize covariance matrix
    for(int i = 0; i < 13; i++) {
        for(int j = 0; j < 13; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Higher uncertainty for velocity and acceleration
    for(int i = 0; i < 6; i++) {
        P[i][i] = 1.0f;
    }
    // Lower uncertainty for quaternion
    for(int i = 6; i < 10; i++) {
        P[i][i] = 0.1f;
    }
    // Low uncertainty for angular velocity
    for(int i = 10; i < 13; i++) {
        P[i][i] = 0.1f;
    }
    
    initialized = true;
}

void GY85_KalmanFilter::update(float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z,
                                float mag_x, float mag_y, float mag_z,
                                float dt) {
    if(!initialized) {
        init();
    }
    
    // Convert sensor readings from FRD (Forward-Right-Down) to FLU (Forward-Left-Up)
    // FRD: [x_fwd, y_right, z_down]
    // FLU: [x_fwd, y_left, z_up]
    // Transformation: FLU_x = FRD_x, FLU_y = -FRD_y, FLU_z = -FRD_z
    float gyro[3] = {gyro_x, -gyro_y, -gyro_z};           // rad/s in FLU
    float accel_body[3] = {accel_x, -accel_y, -accel_z}; // m/s^2 in FLU
    float mag[3] = {mag_x, -mag_y, -mag_z};              // uT in FLU
    
    // Prediction step
    predictState(dt, gyro, accel_body);
    
    // Update step
    updateWithAccel(accel_body);
    updateWithMag(mag);
    updateWithGyro(gyro);
}

void GY85_KalmanFilter::predictState(float dt, float gyro[3], float accel_body[3]) {
    // Extract state components
    float q[4] = {state[6], state[7], state[8], state[9]};
    float wx = state[10], wy = state[11], wz = state[12];
    
    // Update quaternion using gyro (body frame angular velocity)
    float dq[4];
    dq[0] = 0.5f * (-state[7]*wx - state[8]*wy - state[9]*wz);
    dq[1] = 0.5f * (state[6]*wx + state[8]*wz - state[9]*wy);
    dq[2] = 0.5f * (state[6]*wy - state[7]*wz + state[9]*wx);
    dq[3] = 0.5f * (state[6]*wz + state[7]*wy - state[8]*wx);
    
    for(int i = 0; i < 4; i++) {
        state[6 + i] += dq[i] * dt;
    }
    
    // Normalize quaternion
    quaternionNormalize(&state[6]);
    
    // Transform body acceleration to world frame and compensate gravity
    float accel_world[3];
    compensateGravity(accel_body, &state[6], accel_world);
    
    // Update acceleration (filtered)
    for(int i = 0; i < 3; i++) {
        state[3 + i] = accel_world[i];
    }
    
    // Update velocity by integrating acceleration
    for(int i = 0; i < 3; i++) {
        state[i] += state[3 + i] * dt;
    }
    
    // Angular velocity update (direct from gyro, will be filtered in update step)
    for(int i = 0; i < 3; i++) {
        state[10 + i] = gyro[i];
    }
    
    // Update process noise in covariance matrix
    for(int i = 0; i < 3; i++) {
        P[i][i] += Q_accel * dt * dt;           // velocity
        P[3+i][3+i] += Q_accel * dt;            // acceleration
        P[6+i][6+i] += Q_attitude * dt;         // attitude
        P[10+i][10+i] += Q_gyro * dt;           // angular velocity
    }
}

void GY85_KalmanFilter::updateWithAccel(float accel_body[3]) {
    float accel_norm = sqrt(accel_body[0]*accel_body[0] + 
                           accel_body[1]*accel_body[1] + 
                           accel_body[2]*accel_body[2]);
    
    if(accel_norm < 0.5f) return;
    
    // Normalize acceleration measurement
    float accel_n[3];
    for(int i = 0; i < 3; i++) {
        accel_n[i] = accel_body[i] / accel_norm;
    }
    
    // Expected gravity in body frame
    float q[4] = {state[6], state[7], state[8], state[9]};
    float gravity_world[3] = {0.0f, 0.0f, -9.81f};
    float gravity_body[3];
    
    rotateVectorByQuaternionInv(q, gravity_world, gravity_body);
    
    float gravity_norm = sqrt(gravity_body[0]*gravity_body[0] + 
                             gravity_body[1]*gravity_body[1] + 
                             gravity_body[2]*gravity_body[2]);
    
    if(gravity_norm > 0.1f) {
        for(int i = 0; i < 3; i++) {
            gravity_body[i] /= gravity_norm;
        }
    }
    
    // Residual between measurement and predicted gravity
    float residual[3];
    for(int i = 0; i < 3; i++) {
        residual[i] = accel_n[i] - gravity_body[i];
    }
    
    // Simple complementary filter update to attitude
    float K = 0.05f;
    state[7] += K * residual[1];  // pitch correction
    state[8] += K * (-residual[0]); // roll correction
    
    quaternionNormalize(&state[6]);
}

void GY85_KalmanFilter::updateWithMag(float mag[3]) {
    float mag_norm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    
    if(mag_norm < 0.1f) return;
    
    // Normalize magnetic field measurement
    float mag_n[3];
    for(int i = 0; i < 3; i++) {
        mag_n[i] = mag[i] / mag_norm;
    }
    
    // Transform magnetic field to world frame
    float q[4] = {state[6], state[7], state[8], state[9]};
    float mag_world[3];
    rotateVectorByQuaternion(q, mag_n, mag_world);
    
    // Calculate yaw from magnetic field
    float yaw_mag = atan2(mag_world[1], mag_world[0]);
    
    // Get current yaw from quaternion
    float roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    
    // Calculate yaw error
    float yaw_error = yaw_mag - yaw;
    
    // Wrap yaw error to [-pi, pi]
    while(yaw_error > PI) yaw_error -= 2.0f*PI;
    while(yaw_error < -PI) yaw_error += 2.0f*PI;
    
    // Update quaternion yaw component
    float K_yaw = 0.02f;
    state[9] += K_yaw * yaw_error * state[6];
    
    quaternionNormalize(&state[6]);
}

void GY85_KalmanFilter::updateWithGyro(float gyro[3]) {
    // Low-pass filter on gyro measurements
    float K_gyro = 0.95f;
    for(int i = 0; i < 3; i++) {
        state[10 + i] = K_gyro * state[10 + i] + (1.0f - K_gyro) * gyro[i];
    }
}

void GY85_KalmanFilter::getVelocity(float &vx, float &vy, float &vz) {
    vx = state[0];
    vy = state[1];
    vz = state[2];
}

void GY85_KalmanFilter::getAcceleration(float &ax, float &ay, float &az) {
    ax = state[3];
    ay = state[4];
    az = state[5];
}

void GY85_KalmanFilter::getAttitude(float &roll, float &pitch, float &yaw) {
    float q[4] = {state[6], state[7], state[8], state[9]};
    quaternionToEuler(q, roll, pitch, yaw);
}

void GY85_KalmanFilter::getAngularVelocity(float &wx, float &wy, float &wz) {
    wx = state[10];
    wy = state[11];
    wz = state[12];
}

void GY85_KalmanFilter::setProcessNoise(float q_accel, float q_gyro, float q_attitude) {
    Q_accel = q_accel;
    Q_gyro = q_gyro;
    Q_attitude = q_attitude;
}

void GY85_KalmanFilter::setMeasurementNoise(float r_accel, float r_mag, float r_gyro) {
    R_accel = r_accel;
    R_mag = r_mag;
    R_gyro = r_gyro;
}

void GY85_KalmanFilter::reset() {
    initialized = false;
    init();
}

void GY85_KalmanFilter::quaternionNormalize(float q[4]) {
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm > 0.0001f) {
        for(int i = 0; i < 4; i++) {
            q[i] /= norm;
        }
    }
}

void GY85_KalmanFilter::quaternionToEuler(float q[4], float &roll, float &pitch, float &yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if(abs(sinp) >= 1.0f)
        pitch = copysign(PI / 2.0f, sinp);
    else
        pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void GY85_KalmanFilter::quaternionMultiply(float q1[4], float q2[4], float result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void GY85_KalmanFilter::rotateVectorByQuaternion(float q[4], float v[3], float result[3]) {
    // v' = q * v * q^-1
    float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    float q_v[4] = {0.0f, v[0], v[1], v[2]};
    float temp[4], final[4];
    
    quaternionMultiply(q, q_v, temp);
    quaternionMultiply(temp, q_conj, final);
    
    result[0] = final[1];
    result[1] = final[2];
    result[2] = final[3];
}

void GY85_KalmanFilter::rotateVectorByQuaternionInv(float q[4], float v[3], float result[3]) {
    // v' = q^-1 * v * q
    float q_inv[4] = {q[0], -q[1], -q[2], -q[3]};
    float q_v[4] = {0.0f, v[0], v[1], v[2]};
    float temp[4], final[4];
    
    quaternionMultiply(q_inv, q_v, temp);
    quaternionMultiply(temp, q, final);
    
    result[0] = final[1];
    result[1] = final[2];
    result[2] = final[3];
}

void GY85_KalmanFilter::compensateGravity(float accel_body[3], float q[4], float accel_world[3]) {
    // Transform body acceleration to world frame
    rotateVectorByQuaternion(q, accel_body, accel_world);
    
    // Compensate gravity (z-axis pointing up)
    accel_world[2] -= 9.81f;
}

#endif