//The code is translated from arduino code available in the below link
//https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

//imu settings
//Accl range +-2g
//Gyro range +-250deg/s
#include "simpletools.h"
#include "simplei2c.h"
#include "mstimer.h"

#define MPU6050_ADDR 0x68  // Default I2C address (AD0 pin low)
#define SCL_PIN 6          // Use P0 for SCL
#define SDA_PIN 7          // Use P1 for SDA

i2c imu;  // I2C bus for MPU-60X0

// Register addresses from the datasheet
#define PWR_MGMT_1     0x6B 
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43
#define PI 3.148

// Sensor data structure
typedef struct {
    float accel[3];  // X,Y,Z
    float gyro[3];   // X,Y,Z
    float temp;
} imu_data_t;

// Initialize MPU-60X0
void mpu6050_init() {
    // Initialize I2C
    i2c_open(&imu, SCL_PIN, SDA_PIN, 0);
    
    // Wake up MPU-60X0 (disable sleep mode)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    i2c_writeByte(&imu, 0x80);  // Set DEVICE_RESET bit
    i2c_stop(&imu);
    
    // Wait for reset to complete
    pause(100);
    
    // 2. Wake up MPU-6050 and configure clock source
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    // CLKSEL = 1 (PLL with X gyro reference)
    // Also clears SLEEP bit and TEMP_DIS bit
    i2c_writeByte(&imu, 0x01);  
    i2c_stop(&imu);
    
    // Configure gyroscope range (±250°/s)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, GYRO_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±250°/s (FS_SEL=0)
    i2c_stop(&imu);
    
    // Configure accelerometer range (±2g)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±2g (AFS_SEL=0)
    i2c_stop(&imu);
   
}

// Read all sensor data from MPU-60X0
void mpu6050_read(imu_data_t *data) {
    uint8_t buf[14];
    
    // Start reading from register 0x3B (ACCEL_XOUT_H)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_XOUT_H);
    
    // Repeated start to begin reading
    i2c_start(&imu);
    i2c_writeByte(&imu, (MPU6050_ADDR << 1) | 1);
    
    // Read 14 bytes (accel, temp, gyro)
    for(int i = 0; i < 13; i++) {
       
        buf[i] = i2c_readByte(&imu, 0);  // ACK all but last byte
  
    }
    buf[13] = i2c_readByte(&imu, 1);     // NAK last byte
    i2c_stop(&imu);
    
    // Format data (registers are big-endian)
    data->accel[0] = ((buf[0] << 8) | buf[1]) / 16384.0 ;   // X
    data->accel[1] = ((buf[2] << 8) | buf[3]) / 16384.0 ;   // Y
    data->accel[2] = ((buf[4] << 8) | buf[5]) / 16384.0 ;   // Z
    data->temp     = (buf[6] << 8) | buf[7];   // Temperature
    data->gyro[0]  = ((buf[8] << 8) | buf[9]) / 131.0;   // X
    data->gyro[1]  = ((buf[10] << 8) | buf[11])/ 131.0; // Y
    data->gyro[2]  = ((buf[12] << 8) | buf[13])/ 131.0; // Z
}

// Calibrate gyroscope by averaging samples while stationary
void calibrate_gyro(int samples, int16_t *gyro_offsets, int16_t *accel_offsets) {
    imu_data_t data;
    
    float gyro_sum[3] = {0};
    float accel_sum[3] = {0};
    
    print("Calibrating gyro... keep sensor still!\n");
    
    for(int i = 0; i < samples; i++) {
        mpu6050_read(&data);
        gyro_sum[0] += data.gyro[0];
        gyro_sum[1] += data.gyro[1];
        gyro_sum[2] += data.gyro[2];
        accel_sum[0] = accel_sum[0] + ((atan((accel_sum[1]) / sqrt(pow((accel_sum[0]), 2) + pow((accel_sum[2]), 2))) * 180 / PI));
        accel_sum[1] = accel_sum[1] + ((atan(-1 * (accel_sum[0]) / sqrt(pow((accel_sum[1]), 2) + pow((accel_sum[2]), 2))) * 180 / PI));
        
        pause(10);
    }
    
    gyro_offsets[0] = gyro_sum[0] / samples;
    gyro_offsets[1] = gyro_sum[1] / samples;
    gyro_offsets[2] = gyro_sum[2] / samples;
    accel_offsets[0] = accel_sum[0]/ samples;
    accel_offsets[1] = accel_sum[1]/ samples;
    print("gx = %d gy = %d gz = %d \n\n ax = %d ay = %d\n",gyro_offsets[0],gyro_offsets[1],gyro_offsets[2],accel_offsets[0],accel_offsets[1]);
    print("Offsets: X=%d, Y=%d, Z=%d\n", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2],accel_offsets[0],accel_offsets[1]);
    pause(5000);
}

int main() {
    // Initialize IMU
    mpu6050_init();
    mstime_start();
    float yaw = 0,pitch =0, roll = 0;
    // Calibrate gyroscope
    float gyro_offsets[3] = {0};
    float accel_offsets[3] = {0};
    calibrate_gyro(200, gyro_offsets,accel_offsets);
    int16_t prev_time = 0;
    int16_t elapsed_time = 0; 
    int16_t current_time = 0;
    // Main loop
    while(1) {
        imu_data_t data;
        mpu6050_read(&data);
        prev_time = current_time;
        waitcnt(CNT += CLKFREQ/1000);
        current_time = mstime_get();
        elapsed_time = current_time-prev_time;
        // Apply gyro calibration
        data.gyro[0] -= gyro_offsets[0];
        data.gyro[1] -= gyro_offsets[1];
        data.gyro[2] -= gyro_offsets[2];
        
        // Convert raw data to human-readable values:
        // Accelerometer: ±2g range (16384 LSB/g)
        
        float ax = data.accel[0];
        float ay = data.accel[1]; 
        float az = data.accel[2] ;
        float ax_angle = (atan(data.accel[1] / sqrt(pow(data.accel[0], 2) + pow(data.accel[2], 2))) * 180 / PI) - accel_offsets[0];
        float ay_angle =  (atan(-1 * data.accel[0] / sqrt(pow(data.accel[1], 2) + pow(data.accel[2], 2))) * 180 / PI) - accel_offsets[0];
        // Gyroscope: ±250°/s range (131 LSB/°/s)
        float gx = data.gyro[0] ;
        float gy = data.gyro[1] ;
        float gz = data.gyro[2] ;
        
        
        yaw = yaw + gz*elapsed_time;
        roll = 0.96 * gx + 0.04*ax;
        pitch = 0.96*gy + 0.04*ay;
        
        
        
        // Temperature: (in °C)
        float temp = data.temp / 340.0 + 36.53;
        
        // Print results with fixed-width formatting
        //print("%c", HOME);  // Clear terminal
        print("MPU-60X0 IMU Data\n");
        print("-----------------\n");
        print("Accel: X=%7.2fg  Y=%7.2fg  Z=%7.2fg\n", ax, ay, az);
        print("elapsed_time = %d\n",elapsed_time);
        print("Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", gx, gy, gz);
        print("Roll =%7.2f°/s Pitch=%7.2f°/s Yaw=%7.2f°/s\n", roll, pitch, yaw);
        //print("Temp:  %7.1f°C\n", temp);
        while(1){pause(200);}        
    }      
}