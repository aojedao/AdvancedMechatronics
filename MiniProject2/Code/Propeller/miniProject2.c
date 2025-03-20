#include "simpletools.h"
#include "simplei2c.h"
#include "fdserial.h"
#include "servo.h"

// Constants
#define WHEEL_RADIUS 0.05          // Meters
#define WHEEL_DISTANCE 0.15        // Meters between wheels
#define KP 1.0                      // PID gains
#define KI 0.5
#define KD 0.2

// Motor driver pins (TB6612)
#define PWMA 4
#define AIN1 5
#define AIN2 6
#define PWMB 7
#define BIN1 8
#define BIN2 9
#define STBY 10

// SPI pins (IMU)
#define MOSI 0
#define MISO 1
#define SCLK 2
#define CS   3

// IMU registers (MPU-9250)
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_ZOUT_H 0x47

// BLE
fdserial *ble;

// Variables
volatile int wheel_pos[2] = {0, 0};   // Encoder positions
volatile float x_pos = 0.0, y_pos = 0.0, theta = 0.0;   // Robot position
volatile int speed[2] = {50, 50};     // Motor speeds
volatile float imu_data[6];           // IMU data: accel X/Y/Z, gyro X/Y/Z

// Function prototypes
void motor_control();
void encoder_reading();
void imu_processing();
void sensor_fusion();
void ble_communication();
void handle_command(char *cmd);
void move(int left, int right);
void stop();


// SPI Communication Functions
void spi_init() {
    low(CS);
    set_direction(MOSI, 1);
    set_direction(MISO, 0);
    set_direction(SCLK, 1);
    set_direction(CS, 1);
}

uint8_t spi_transfer(uint8_t data) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            high(MOSI);
        } else {
            low(MOSI);
        }
        high(SCLK);
        result <<= 1;
        if (input(MISO)) {
            result |= 0x01;
        }
        low(SCLK);
        data <<= 1;
    }
    return result;
}

void mpu_write(uint8_t reg, uint8_t data) {
    low(CS);
    spi_transfer(reg & 0x7F);
    spi_transfer(data);
    high(CS);
}

uint8_t mpu_read(uint8_t reg) {
    low(CS);
    spi_transfer(reg | 0x80);
    uint8_t result = spi_transfer(0x00);
    high(CS);
    return result;
}

void readIMU() {
    uint8_t buffer[6];

    // Read gyroscope data (Z-axis rotation)
    buffer[0] = mpu_read(0x47);  // GYRO_ZOUT_H
    buffer[1] = mpu_read(0x48);  // GYRO_ZOUT_L
    int16_t gyro_z = (int16_t)((buffer[0] << 8) | buffer[1]);
    imu_data[2] = gyro_z / 131.0;  // Gyro Z in degrees/sec
}

// Initialize the TB6612 motor driver
void motor_init() {
    high(STBY);   // Enable the motor driver
}

// Move the motors with PWM
void move(int left, int right) {
    pwm_set(PWMA, 0, left > 0 ? left : -left);
    pwm_set(PWMB, 1, right > 0 ? right : -right);

    // Set motor direction
    high(left > 0 ? AIN1 : AIN2);
    low(left > 0 ? AIN2 : AIN1);
    
    high(right > 0 ? BIN1 : BIN2);
    low(right > 0 ? BIN2 : BIN1);
}

// Stop the motors
void stop() {
    low(PWMA);
    low(PWMB);
}

// Main function
int main() {
    // Initialize BLE
    ble = fdserial_open(10, 11, 0, 115200);

    // Start cogs
    cog_run(&motor_control, 128);
    cog_run(&encoder_reading, 128);
    cog_run(&imu_processing, 128);
    cog_run(&sensor_fusion, 128);
    cog_run(&ble_communication, 128);

    // Main loop
    while (1) {
        pause(500);
    }
}

// Motor control with PID
void motor_control() {
    motor_init();
    while (1) {
        // Simple PID control loop
        int error[2] = {speed[0] - wheel_pos[0], speed[1] - wheel_pos[1]};
        move(error[0], error[1]);
        pause(20);
    }
}

// Encoder simulation (for testing)
void encoder_reading() {
    while (1) {
        wheel_pos[0] += rand() % 5;  // Simulated encoder readings
        wheel_pos[1] += rand() % 5;
        pause(50);
    }
}

// IMU processing
void imu_processing() {
    while (1) {
        readIMU();
        pause(100);
    }
}

// Sensor fusion
void sensor_fusion() {
    while (1) {
        float ds = ((wheel_pos[0] + wheel_pos[1]) / 2.0) * WHEEL_RADIUS;
        float dtheta = ((wheel_pos[0] - wheel_pos[1]) / WHEEL_DISTANCE);

        float alpha = 0.98;
        float imu_yaw = imu_data[5];

        theta = alpha * (theta + dtheta) + (1 - alpha) * imu_yaw;
        x_pos += ds * cos(theta);
        y_pos += ds * sin(theta);

        pause(50);
    }
}

// BLE communication
void ble_communication() {
    char buf[32];
    while (1) {
        if (fdserial_rxCount(ble) > 0) {
            fdserial_rxFlush(ble);
            fdserial_rx(ble);  
            handle_command(buf);
        }

        dprint(ble, "POS: X:%.2f Y:%.2f Th:%.2f\n", x_pos, y_pos, theta);
        dprint(ble, "IMU: %.2f %.2f %.2f\n", imu_data[0], imu_data[1], imu_data[2]);
        pause(500);
    }
}

// Handle BLE commands
void handle_command(char *cmd) {
    if (strcmp(cmd, "FORWARD") == 0) move(50, 50);
    if (strcmp(cmd, "BACKWARD") == 0) move(-50, -50);
    if (strcmp(cmd, "STOP") == 0) stop();
}
