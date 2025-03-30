//The code is translated from arduino code available in the below link
//https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/.....
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

typedef struct{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
  
} received_data;

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
    int16_t buf[14];
    
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
    received_data R_Data;
    R_Data.accel[0] = ((buf[0] << 8) | buf[1]);
    data->accel[0] = (float)(R_Data.accel[0]) / 16384.0 ;   // X
    R_Data.accel[1] = ((buf[2] << 8) | buf[3]);
    data->accel[1] = (float)(R_Data.accel[1]) / 16384.0 ;   // Y
    R_Data.accel[2] = ((buf[4] << 8) | buf[5]);
    data->accel[2] = (float)(R_Data.accel[2]) / 16384.0 ;   // Z
    R_Data.temp = (buf[6] << 8) | buf[7];
    data->temp     =  (float)(R_Data.temp);  // Temperature
    R_Data.gyro[0]  = (buf[8] << 8) | buf[9];   // X
    data->gyro[0] = (float)(R_Data.gyro[0])/131.0;
    R_Data.gyro[1]  = (buf[10] << 8) | buf[11];   // X
    data->gyro[1] = (float)(R_Data.gyro[1])/131.0;
    R_Data.gyro[2]  = (buf[12] << 8) | buf[13];   // X
    data->gyro[2] = (float)(R_Data.gyro[2])/131.0;
}

// Calibrate gyroscope by averaging samples while stationary
void calibrate_gyro(int samples, float *gyro_offsets, float *accel_offsets) {
   print("Calibrating gyro... keep sensor still!\n");

    imu_data_t data;
    
    float gyro_sum[3] = {0};
    float accel_sum[3] = {0};
    
    
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
    pause(500);
}

int main() {
    // Initialize IMU
    mpu6050_init();
    
    float yaw = 0,pitch =0, roll = 0;
    float   gyroAngleX = 0, gyroAngleY = 0;
    // Calibrate gyroscope
    float gyro_offsets[3] = {0};
    float accel_offsets[3] = {0};
    calibrate_gyro(200, gyro_offsets,accel_offsets);
    int16_t prev_time = 0;
    int16_t elapsed_time = 0; 
    int16_t current_time = 0;
    // Main loop
    print("finished calibration\n");
    
    mstime_start();
    while(1) {
        imu_data_t data;
        //mstime_reset();
       // int16_t t1 = mstime_get();
        prev_time = current_time;
        mpu6050_read(&data);
        //int16_t t2 = mstime_get();
        
        waitcnt(CNT += CLKFREQ/1000);
        current_time = mstime_get();
        elapsed_time = (current_time - prev_time)/1000;
        
        
        // Apply gyro calibration
        int16_t t3 = mstime_get();
        data.gyro[0] -= gyro_offsets[0];
        data.gyro[1] -= gyro_offsets[1];
        data.gyro[2] -= gyro_offsets[2];
        
        
        // Convert raw data to human-readable values:
        // Accelerometer: ±2g range (16384 LSB/g)
        
        float ax = data.accel[0]*1;
        float ay = data.accel[1]*1; 
        float az = data.accel[2]*1 ;
        float ax_angle = (atan2(data.accel[1] , sqrtf(pow(data.accel[0], 2) + pow(data.accel[2], 2))) * 180 / PI) - accel_offsets[0];
        float ay_angle =  (atan2(-1 * data.accel[0] , sqrtf(pow(data.accel[1], 2) + pow(data.accel[2], 2))) * 180 / PI) - accel_offsets[0];
        // Gyroscope: ±250°/s range (131 LSB/°/s)
        float gx = data.gyro[0] ;
        float gy = data.gyro[1] ;
        float gz = data.gyro[2] ;
        
        gyroAngleX = gyroAngleX + gx * elapsed_time;
        gyroAngleY = gyroAngleY + gy * elapsed_time;
        yaw = yaw + gz*elapsed_time;
        roll = 0.96 * gx + 0.04*ax;
        pitch = 0.96*gy + 0.04*ay;
        
        
        
        // Temperature: (in °C)
        float temp = data.temp / 340.0 + 36.53;
        int16_t t4 = mstime_get();
        // Print results with fixed-width formatting
        print("%c", HOME);  // Clear terminal
        print("MPU-60X0 IMU Data\n");
        print("-----------------\n");
        print("Accel: X=%7.2fg  Y=%7.2fg  Z=%7.2fg\n", ax, ay, az);
        print("Clockfreq = %d\t elapsed_time = %d\n",CLKFREQ,elapsed_time);
        print("Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", gx, gy, gz);
        print("Roll =%7.2f°/s Pitch=%7.2f°/s Yaw=%7.2f°/s\n", roll, pitch, yaw);
        //print("Temp:  %7.1f°C\n", temp);
        //while(1){pause(500);}
        pause(1000);
        print("%c",HOME);
    }      
}