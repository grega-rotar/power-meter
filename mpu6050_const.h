// for MPU-6050
// values for 250deg/s
// I2C podrobnosti
#define MPU_ADDR 0x68


// #define BIAS_GYRO_X -7651
// #define BIAS_GYRO_Y 545
// #define BIAS_GYRO_Z 80
#define BIAS_GYRO_X -66
#define BIAS_GYRO_Y 26
#define BIAS_GYRO_Z 52

#define BIAS_ACCEL_X 357.108853
#define BIAS_ACCEL_Y -23.930205
#define BIAS_ACCEL_Z 131.861592

// gyroscope range
// FS_SEL Full Scale Range LSB Sensitivity
// 0 ± 250 °/s 131 LSB/°/s
// 1 ± 500 °/s 65.5 LSB/°/s
// 2 ± 1000 °/s 32.8 LSB/°/s
// 3 ± 2000 °/s 16.4 LSB/°/s
#define FS_SEL 3
#if FS_SEL == 0 // ± 250 °/s
#define LSB_DEG 131.0
#elif FS_SEL == 1 // ± 500 °/s
#define LSB_DEG 65.5
#elif FS_SEL == 2 // ± 1000 °/s
#define LSB_DEG 32.8
#elif FS_SEL == 3 // ± 2000 °/s
#define LSB_DEG 16.4
#endif
#define LSB_DEG_CALIBRATED 16.4 // setting / value used when calibrating nof LSB

// accelometer range
// AFS_SEL Full Scale Range LSB Sensitivity
// 0 ±2g 16384 LSB/g
// 1 ±4g 8192 LSB/g
// 2 ±8g 4096 LSB/g
// 3 ±16g 2048 LSB/g
#define AFS_SEL 2
#if AFS_SEL == 0 // ±2g
#define LSB_G 16384.0
#elif AFS_SEL == 1 // ±4g
#define LSB_G 8192.0
#elif AFS_SEL == 2 // ±8g
#define LSB_G 4096.0
#elif AFS_SEL == 3 // ±16g
#define LSB_G 2048.0
#endif
#define LSB_G_CALIBRATED 4096.0 // setting / value used when calibrating