#ifndef IMU_H
#define IMU_H

rc_imu_data_t imuData;

//set config to default to prevent error through forgetting to set a value
rc_imu_config imuConfig = rc_default_imu_config();

//-----Config variables-----
//Accelerometer and Gyro Settings
imuConfig.accel_fsr = A_FSR_4G; //accelerometer sensitivity
imuConfig.gyro_fsr = G_FSR_1000DPS; //gyro sensitivity
imuConfig.accel_dlpf = ACCEL_DLPF_OFF; //accelerometer low pass filter
imuConfig.gyro_dlpf = GYRO_DLPF_OFF; //gyro low pass filter
imuConfig.orientation = ORIENTATION_Z_UP; //orientation

//Compass Settings
imuConfig.enable_magnetormeter = 0;  //1 is enabled
//imuConfig.compass_time_constant = 1.0; //not used

//General Settings
imuConfig.dmp_sample_rate = 10;
//imuConfig.dmp_interrupt_priority;
imuConfig.show_warnings = 0; //1 shows i2c bus warnings




#endif // IMU_H
