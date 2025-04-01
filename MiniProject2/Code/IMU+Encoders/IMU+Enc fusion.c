#include "simpletools.h"
#include "simplei2c.h"
#include "mstimer.h"

// IMU Configuration
#define MPU6050_ADDR 0x68
#define SCL_PIN 6
#define SDA_PIN 7
i2c imu;

// IMU Registers
#define PWR_MGMT_1     0x6B 
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43
#define PI 3.14159265359

// Encoder Configuration
#define RIGHT_A 11  
#define RIGHT_B 12  
#define LEFT_A  9
#define LEFT_B  10

// Constants
#define WHEEL_DIAMETER 0.065     // meters
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define COUNTS_PER_REV (48 * 4)  // 48 CPR with quadrature decoding
#define GEAR_RATIO 4.88          // Calculated from your data
#define COUNTS_PER_WHEEL_REV (COUNTS_PER_REV * GEAR_RATIO)
#define METERS_PER_COUNT (WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV)

// Global variables
static volatile int32_t rightWheelCount = 0;
static volatile int32_t leftWheelCount = 0;
static unsigned int encoderStack[128*2];

// IMU data structures
typedef struct {
    float accel[3];  // X,Y,Z
    float gyro[3];   // X,Y,Z
    float temp;
} imu_data_t;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
} received_data;

// Odometry data structure
typedef struct {
    float x;        // X position (meters)
    float y;        // Y position (meters)
    float theta;    // Heading (radians)
    float velocity; // Linear velocity (m/s)
    float omega;    // Angular velocity (rad/s)
} odometry_t;

// Encoder decoder lookup table
static const int8_t decoderLookup[16] = {
    0, +1, -1, +2, 
    -1, 0, -2, +1, 
    +1, -2, 0, -1, 
    +2, -1, +1, 0   
};

// Encoder cog function
void encoderCog(void *par) {
    uint8_t lastRight = 0;
    uint8_t lastLeft = 0;
    
    while(1) {
        uint8_t currRight = (input(RIGHT_B) << 1) | input(RIGHT_A);
        uint8_t currLeft = (input(LEFT_B) << 1) | input(LEFT_A);
        
        uint8_t rightIndex = (currRight << 2) | lastRight;
        uint8_t leftIndex = (currLeft << 2) | lastLeft;
        
        rightWheelCount += decoderLookup[rightIndex];
        leftWheelCount += decoderLookup[leftIndex];
        
        lastRight = currRight & 0x03;  
        lastLeft = currLeft & 0x03;
        
        waitcnt(CNT + CLKFREQ/10000);  // 100μs delay
    }
}

// Initialize MPU-60X0
void mpu6050_init() {
    i2c_open(&imu, SCL_PIN, SDA_PIN, 0);
    
    // Wake up MPU-60X0
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    i2c_writeByte(&imu, 0x80);  // Reset
    i2c_stop(&imu);
    pause(100);
    
    // Configure
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    i2c_writeByte(&imu, 0x01);  
    i2c_stop(&imu);
    
    // Gyro config
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, GYRO_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±250°/s
    i2c_stop(&imu);
    
    // Accel config
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±2g
    i2c_stop(&imu);
}

// Read IMU data
void mpu6050_read(imu_data_t *data) {
    int16_t buf[14];
    
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_XOUT_H);
    
    i2c_start(&imu);
    i2c_writeByte(&imu, (MPU6050_ADDR << 1) | 1);
    
    for(int i = 0; i < 13; i++) {
        buf[i] = i2c_readByte(&imu, 0);
    }
    buf[13] = i2c_readByte(&imu, 1);
    i2c_stop(&imu);
    
    received_data R_Data;
    R_Data.accel[0] = ((buf[0] << 8) | buf[1]);
    data->accel[0] = (float)(R_Data.accel[0]) / 16384.0;
    R_Data.accel[1] = ((buf[2] << 8) | buf[3]);
    data->accel[1] = (float)(R_Data.accel[1]) / 16384.0;
    R_Data.accel[2] = ((buf[4] << 8) | buf[5]);
    data->accel[2] = (float)(R_Data.accel[2]) / 16384.0;
    R_Data.temp = (buf[6] << 8) | buf[7];
    data->temp = (float)(R_Data.temp);
    R_Data.gyro[0] = (buf[8] << 8) | buf[9];
    data->gyro[0] = (float)(R_Data.gyro[0])/131.0;
    R_Data.gyro[1] = (buf[10] << 8) | buf[11];
    data->gyro[1] = (float)(R_Data.gyro[1])/131.0;
    R_Data.gyro[2] = (buf[12] << 8) | buf[13];
    data->gyro[2] = (float)(R_Data.gyro[2])/131.0;
}

// Calibrate IMU
void calibrate_imu(int samples, float *gyro_offsets, float *accel_offsets) {
    print("Calibrating gyro... keep sensor still!\n");
    imu_data_t data;
    float gyro_sum[3] = {0};
    float accel_sum[3] = {0};
    
    for(int i = 0; i < samples; i++) {
        mpu6050_read(&data);
        print("i = %d\n",i);
        gyro_sum[0] += data.gyro[0];
        gyro_sum[1] += data.gyro[1];
        gyro_sum[2] += data.gyro[2];
        accel_sum[0] += ((atan((accel_sum[1]) / sqrt(pow((accel_sum[0]), 2) + pow((accel_sum[2]), 2))) * 180 / PI));
        accel_sum[1] += ((atan(-1 * (accel_sum[0]) / sqrt(pow((accel_sum[1]), 2) + pow((accel_sum[2]), 2))) * 180 / PI));
        pause(10);
    }
    
    gyro_offsets[0] = gyro_sum[0] / samples;
    gyro_offsets[1] = gyro_sum[1] / samples;
    gyro_offsets[2] = gyro_sum[2] / samples;
    accel_offsets[0] = accel_sum[0]/ samples;
    accel_offsets[1] = accel_sum[1]/ samples;
    pause(500);
    print("\n Finished Calibration\n");
}

// Update odometry using encoder and IMU data
void update_odometry(odometry_t *odom, imu_data_t *imu_data, int32_t rightCount, int32_t leftCount, float dt) {
    static int32_t prevRightCount = 0;
    static int32_t prevLeftCount = 0;
    
    // Calculate distance traveled by each wheel
    float rightDist = (rightCount - prevRightCount) * METERS_PER_COUNT;
    float leftDist = (leftCount - prevLeftCount) * METERS_PER_COUNT;
    
    prevRightCount = rightCount;
    prevLeftCount = leftCount;
    
    
    // Calculate linear and angular displacement from encoders
    float linear_dist = (rightDist + leftDist) / 2.0;
    float angular_dist = (rightDist - leftDist) / WHEEL_DIAMETER;
    
    
    // Get angular velocity from IMU (more reliable for short-term changes)
    float imu_omega = imu_data->gyro[2] * PI / 180.0; // Convert to rad/s
    
    // Complementary filter: Use IMU for high-frequency changes and encoders for low-frequency
    static float theta = 0;
    float alpha = 0.98; // Weight for IMU data
    
    theta += imu_omega * dt; // IMU integration
    theta = alpha * theta + (1-alpha) * angular_dist; // Combine with encoder data
    
    // Update position
    odom->x += linear_dist * cos(theta);
    odom->y += linear_dist * sin(theta);
    odom->theta = theta;
    
    // Update velocities
    print("%f\t %f\n",linear_dist,dt);
    odom->velocity = linear_dist / dt;
    
    odom->omega = imu_omega;
    print("%f\n",odom->velocity);
}

int main() {
    // Initialize hardware
    clkset(_CLKMODE, _CLKFREQ);
    mpu6050_init();
    
    // Set encoder pins as inputs
    set_direction(RIGHT_A, 0);
    set_direction(RIGHT_B, 0);
    set_direction(LEFT_A, 0);
    set_direction(LEFT_B, 0);
    
    // Start encoder cog
    cogstart(&encoderCog, NULL, encoderStack, sizeof(encoderStack));
    
    // Calibrate IMU
    float gyro_offsets[3] = {0};
    float accel_offsets[3] = {0};
    
    calibrate_imu(2, gyro_offsets, accel_offsets);
    // Initialize odometry
    odometry_t odom = {0, 0, 0, 0, 0};
    imu_data_t imu_data;
    int16_t prev_time = mstime_get();
    
    // Main loop
    while(1) {
        int16_t current_time = mstime_get();
        float dt = (current_time - prev_time) / 1000.0; // Convert to msseconds
        prev_time = current_time;
        
        // Read IMU
        mpu6050_read(&imu_data);
        imu_data.gyro[0] -= gyro_offsets[0];
        imu_data.gyro[1] -= gyro_offsets[1];
        imu_data.gyro[2] -= gyro_offsets[2];
        
        // Update odometry
        update_odometry(&odom, &imu_data, rightWheelCount, leftWheelCount, dt);
        print("right wheel counts = %d\n", rightWheelCount);
        /*
        // Display results
        print("%c", HOME);  // Clear terminal
        print("Odometry Data\n");
        print("-------------\n");
        print("Position: X=%.2fm Y=%.2fm\n", odom.x, odom.y);
        print("Heading: %.1f°\n", odom.theta * 180.0 / PI);
        print("Velocity: %.2fm/s Omega=%.2frad/s\n", odom.velocity, odom.omega);
        print("Encoders: R=%d L=%d\n", rightWheelCount, leftWheelCount);
        */
        pause(100);
    }
}