#ifndef GY87_KALMAN_FILTER_H
#define GY87_KALMAN_FILTER_H

#include <Arduino.h>

class GY87_KalmanFilter {
public:
    GY87_KalmanFilter();
    void init(float initial_altitude = 0.0f);
    void update(float accel_x, float accel_y, float accel_z,  // m/s^2
                float gyro_x, float gyro_y, float gyro_z,  // rad/s
                float mag_x, float mag_y, float mag_z,  // uT
                float altitude,  // m
                float dt);  // s
    void getPosition(float &x, float &y, float &z);  // m
    void getVelocity(float &vx, float &vy, float &vz);  // m/s
    void getAcceleration(float &ax, float &ay, float &az);  // m/s^2
    void getAttitude(float &roll, float &pitch, float &yaw);  // rad
    void getAngularVelocity(float &wx, float &wy, float &wz);  // rad/s
    void getAngularAcceleration(float &alpha_x, float &alpha_y, float &alpha_z);  // rad/s^2
    void setProcessNoise(float q_accel, float q_gyro, float q_bias);
    void setMeasurementNoise(float r_accel, float r_mag, float r_alt);
    void reset();
    bool getInitStatus();

private:
    // state vector: [x, y, z, vx, vy, vz, q0, q1, q2, q3, bx, by, bz]
    // q0-q3: quaternion
    // bx-by-bz: gyro bias
    float state[13];
    float P[13][13];  // covariance mat
    float Q_accel;
    float Q_gyro;
    float Q_bias;
    float R_accel;
    float R_mag;
    float R_alt;
    float last_gyro[3];
    float last_accel[3];
    bool initialized;
    void predictState(float dt, float gyro[3], float accel[3], float comp_accel[3]);
    void updateWithAccel(float accel[3]);
    void updateWithMag(float mag[3]);
    void updateWithAltitude(float altitude);
    void quaternionNormalize(float q[4]);
    void quaternionToEuler(float q[4], float &roll, float &pitch, float &yaw);
    void quaternionMultiply(float q1[4], float q2[4], float result[4]);
    void rotateVectorByQuaternion(float q[4], float v[3], float result[3]);
    void compensateGravity(float accel[3], float q[4], float accel_world[3]);
};

GY87_KalmanFilter::GY87_KalmanFilter() {
    Q_accel = 0.01f;
    Q_gyro = 0.001f;
    Q_bias = 0.0001f;
    
    R_accel = 0.1f;
    R_mag = 0.5f;
    R_alt = 1.0f;
    
    initialized = false;
    
    for(int i = 0; i < 3; i++) {
        last_gyro[i] = 0.0f;
        last_accel[i] = 0.0f;
    }
}

bool GY87_KalmanFilter::getInitStatus(){
    return initialized;
}

void GY87_KalmanFilter::init(float initial_altitude) {
    for(int i = 0; i < 13; i++) {
        state[i] = 0.0f;
    }

    state[2] = initial_altitude;  // z = altitude
    // state[6] = 1.0f;  // q0 = 1, means no default rotation
    state[7] = 1.0f;  // q1 = 1, means z-axis pointing down

    for(int i = 0; i < 13; i++) {
        for(int j = 0; j < 13; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for(int i = 0; i < 6; i++) {
        P[i][i] = 10.0f;
    }
    for(int i = 6; i < 10; i++) {
        P[i][i] = 0.1f;
    }
    for(int i = 10; i < 13; i++) {
        P[i][i] = 0.01f;
    }
    
    initialized = true;
}

void GY87_KalmanFilter::update(float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z,
                                float mag_x, float mag_y, float mag_z,
                                float altitude, float dt) {
    if(!initialized) {
        init(altitude);
    }
    
    float gyro[3] = {gyro_x, gyro_y, gyro_z};
    float accel[3] = {accel_x, accel_y, accel_z};
    float mag[3] = {mag_x, mag_y, mag_z};
    float accel_world[3];

    predictState(dt, gyro, accel, accel_world);
    updateWithAccel(accel);
    updateWithMag(mag);
    updateWithAltitude(altitude);

    for(int i = 0; i < 3; i++) {
        last_gyro[i] = gyro[i];
        last_accel[i] = accel_world[i];
    }
}

void GY87_KalmanFilter::predictState(float dt, float gyro[3], float accel[3], float comp_accel[3]) {
    float gyro_corrected[3];
    for(int i = 0; i < 3; i++) {
        gyro_corrected[i] = gyro[i] - state[10 + i];
    }

    float dq[4];
    dq[0] = 0.5f * (-state[7]*gyro_corrected[0] - state[8]*gyro_corrected[1] - state[9]*gyro_corrected[2]);
    dq[1] = 0.5f * (state[6]*gyro_corrected[0] + state[8]*gyro_corrected[2] - state[9]*gyro_corrected[1]);
    dq[2] = 0.5f * (state[6]*gyro_corrected[1] - state[7]*gyro_corrected[2] + state[9]*gyro_corrected[0]);
    dq[3] = 0.5f * (state[6]*gyro_corrected[2] + state[7]*gyro_corrected[1] - state[8]*gyro_corrected[0]);
    
    for(int i = 0; i < 4; i++) {
        state[6 + i] += dq[i] * dt;
    }

    float q[4] = {state[6], state[7], state[8], state[9]};
    quaternionNormalize(q);
    for(int i = 0; i < 4; i++) {
        state[6 + i] = q[i];
    }

    float accel_world[3];
    compensateGravity(accel, q, accel_world);
    //Serial.printf("accel: %.2f, %.2f, %.2f; \n accel_world: %.2f, %.2f, %.2f \n", 
    //    accel[0], accel[1], accel[2], accel_world[0], accel_world[1], accel_world[2]);

    for(int i = 0; i < 3; i++) {
        comp_accel[i] = accel_world[i];
    }

    for(int i = 0; i < 3; i++) {
        state[i] += state[3 + i] * dt + 0.5f * accel_world[i] * dt * dt;
        state[3 + i] += accel_world[i] * dt;
    }

    for(int i = 0; i < 3; i++) {
        P[i][i] += Q_accel * dt * dt * dt * dt / 4.0f;
        P[3+i][3+i] += Q_accel * dt * dt;
        P[6+i][6+i] += Q_gyro * dt;
        P[10+i][10+i] += Q_bias * dt;
    }
}

void GY87_KalmanFilter::updateWithAccel(float accel[3]) {
    float accel_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if(accel_norm < 0.1f) return;

    float accel_n[3];
    for(int i = 0; i < 3; i++) {
        accel_n[i] = accel[i] / accel_norm;
    }

    float gravity_sensor[3];
    float q[4] = {state[6], state[7], state[8], state[9]};
    float gravity_world[3] = {0.0f, 0.0f, -9.81f};

    float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    rotateVectorByQuaternion(q_conj, gravity_world, gravity_sensor);
    
    float gravity_norm = sqrt(gravity_sensor[0]*gravity_sensor[0] + 
                             gravity_sensor[1]*gravity_sensor[1] + 
                             gravity_sensor[2]*gravity_sensor[2]);
    
    if(gravity_norm > 0.1f) {
        for(int i = 0; i < 3; i++) {
            gravity_sensor[i] /= gravity_norm;
        }
    }
    float residual[3];
    for(int i = 0; i < 3; i++) {
        residual[i] = accel_n[i] - gravity_sensor[i];
    }

    float K = 0.05f;

    state[7] += K * residual[1];
    state[8] += K * (-residual[0]);

    quaternionNormalize(&state[6]);
}

void GY87_KalmanFilter::updateWithMag(float mag[3]) {
    float mag_norm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    if(mag_norm < 0.1f) return;
    
    float mag_n[3];
    for(int i = 0; i < 3; i++) {
        mag_n[i] = mag[i] / mag_norm;
    }

    float q[4] = {state[6], state[7], state[8], state[9]};
    float mag_world[3];
    rotateVectorByQuaternion(q, mag_n, mag_world);

    float yaw_mag = atan2(mag_world[1], mag_world[0]);
    float roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    float yaw_error = yaw_mag - yaw;

    while(yaw_error > PI) yaw_error -= 2*PI;
    while(yaw_error < -PI) yaw_error += 2*PI;

    float K_yaw = 0.02f;
    state[9] += K_yaw * yaw_error * state[6];
    
    quaternionNormalize(&state[6]);
}

void GY87_KalmanFilter::updateWithAltitude(float altitude) {
    float innovation = altitude - state[2];
    float S = P[2][2] + R_alt;

    float K[13];
    for(int i = 0; i < 13; i++) {
        K[i] = P[i][2] / S;
    }

    for(int i = 0; i < 13; i++) {
        state[i] += K[i] * innovation;
    }

    for(int i = 0; i < 13; i++) {
        for(int j = 0; j < 13; j++) {
            P[i][j] -= K[i] * P[2][j];
        }
    }
}

void GY87_KalmanFilter::getPosition(float &x, float &y, float &z) {
    x = state[0];
    y = state[1];
    z = state[2];
}

void GY87_KalmanFilter::getVelocity(float &vx, float &vy, float &vz) {
    vx = state[3];
    vy = state[4];
    vz = state[5];
}

void GY87_KalmanFilter::getAcceleration(float &ax, float &ay, float &az) {
    ax = last_accel[0];
    ay = last_accel[1];
    az = last_accel[2];
}

void GY87_KalmanFilter::getAttitude(float &roll, float &pitch, float &yaw) {
    float q[4] = {state[6], state[7], state[8], state[9]};
    quaternionToEuler(q, roll, pitch, yaw);
}

void GY87_KalmanFilter::getAngularVelocity(float &wx, float &wy, float &wz) {
    wx = last_gyro[0] - state[10];
    wy = last_gyro[1] - state[11];
    wz = last_gyro[2] - state[12];
}

void GY87_KalmanFilter::getAngularAcceleration(float &alpha_x, float &alpha_y, float &alpha_z) {
    alpha_x = 0.0f;
    alpha_y = 0.0f;
    alpha_z = 0.0f;
}

void GY87_KalmanFilter::setProcessNoise(float q_accel, float q_gyro, float q_bias) {
    Q_accel = q_accel;
    Q_gyro = q_gyro;
    Q_bias = q_bias;
}

void GY87_KalmanFilter::setMeasurementNoise(float r_accel, float r_mag, float r_alt) {
    R_accel = r_accel;
    R_mag = r_mag;
    R_alt = r_alt;
}

void GY87_KalmanFilter::reset() {
    initialized = false;
    init(state[2]);
}

void GY87_KalmanFilter::quaternionNormalize(float q[4]) {
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(norm > 0.0001f) {
        for(int i = 0; i < 4; i++) {
            q[i] /= norm;
        }
    }
}

void GY87_KalmanFilter::quaternionToEuler(float q[4], float &roll, float &pitch, float &yaw) {
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

void GY87_KalmanFilter::rotateVectorByQuaternion(float q[4], float v[3], float result[3]) {
    float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    float q_v[4] = {0.0f, v[0], v[1], v[2]};
    float temp[4], final[4];
    
    quaternionMultiply(q, q_v, temp);
    quaternionMultiply(temp, q_conj, final);
    
    result[0] = final[1];
    result[1] = final[2];
    result[2] = final[3];
}

void GY87_KalmanFilter::quaternionMultiply(float q1[4], float q2[4], float result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void GY87_KalmanFilter::compensateGravity(float accel[3], float q[4], float accel_world[3]) {
    rotateVectorByQuaternion(q, accel, accel_world);
    accel_world[2] += 9.81f;
}

#endif