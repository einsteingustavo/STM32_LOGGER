// Based on Eugene Gonzalez's version of LSM9DS1_Demo
// Modified by Sherry Yang for LSM6DS3 sensor
#ifndef _LSM6DS3_H__
#define _LSM6DS3_H__

#include "mbed.h"

/////////////////////////////////////////
// LSM6DS3 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define RAM_ACCESS            0x01
#define SENSOR_SYNC_TIME      0x04
#define SENSOR_SYNC_EN        0x05
#define FIFO_CTRL1            0x06
#define FIFO_CTRL2            0x07
#define FIFO_CTRL3            0x08
#define FIFO_CTRL4            0x09
#define FIFO_CTRL5            0x0A
#define ORIENT_CFG_G          0x0B
#define REFERENCE_G           0x0C
#define INT1_CTRL             0x0D
#define INT2_CTRL             0x0E
#define WHO_AM_I_REG          0X0F
#define CTRL1_XL              0x10
#define CTRL2_G               0x11
#define CTRL3_C               0x12
#define CTRL4_C               0x13
#define CTRL5_C               0x14
#define CTRL6_C               0x15
#define CTRL7_G               0x16
#define CTRL8_XL              0x17
#define CTRL9_XL              0x18
#define CTRL10_C              0x19
#define MASTER_CONFIG         0x1A
#define WAKE_UP_SRC           0x1B
#define TAP_SRC               0x1C
#define D6D_SRC               0x1D
#define STATUS_REG            0x1E
#define OUT_TEMP_L            0x20
#define OUT_TEMP_H            0x21
#define OUTX_L_G              0x22
#define OUTX_H_G              0x23
#define OUTY_L_G              0x24
#define OUTY_H_G              0x25
#define OUTZ_L_G              0x26
#define OUTZ_H_G              0x27
#define OUTX_L_XL             0x28
#define OUTX_H_XL             0x29
#define OUTY_L_XL             0x2A
#define OUTY_H_XL             0x2B
#define OUTZ_L_XL             0x2C
#define OUTZ_H_XL             0x2D
#define SENSORHUB1_REG        0x2E
#define SENSORHUB2_REG        0x2F
#define SENSORHUB3_REG        0x30
#define SENSORHUB4_REG        0x31
#define SENSORHUB5_REG        0x32
#define SENSORHUB6_REG        0x33
#define SENSORHUB7_REG        0x34
#define SENSORHUB8_REG        0x35
#define SENSORHUB9_REG        0x36
#define SENSORHUB10_REG       0x37
#define SENSORHUB11_REG       0x38
#define SENSORHUB12_REG       0x39
#define FIFO_STATUS1          0x3A
#define FIFO_STATUS2          0x3B
#define FIFO_STATUS3          0x3C
#define FIFO_STATUS4          0x3D
#define FIFO_DATA_OUT_L       0x3E
#define FIFO_DATA_OUT_H       0x3F
#define TIMESTAMP0_REG        0x40
#define TIMESTAMP1_REG        0x41
#define TIMESTAMP2_REG        0x42
#define STEP_COUNTER_L        0x4B
#define STEP_COUNTER_H        0x4C
#define FUNC_SR               0x53
#define TAP_CFG               0x58
#define TAP_THS_6D            0x59
#define INT_DUR2              0x5A
#define WAKE_UP_THS           0x5B
#define WAKE_UP_DUR           0x5C
#define FREE_FALL             0x5D
#define MD1_CFG               0x5E
#define MD2_CFG               0x5F

// Possible I2C addresses for the accel/gyro
#define LSM6DS3_AG_I2C_ADDR(sa0) ((sa0) ? 0xD6 : 0xD4)

/**
 * LSM6DS3 Class - driver for the 9 DoF IMU
 */
class LSM6DS3
{
public:

    /// gyro_scale defines the possible full-scale ranges of the gyroscope:
    enum gyro_scale
    {
        G_SCALE_245DPS = 0x0 << 3,     // 00 << 3: +/- 245 degrees per second
        G_SCALE_500DPS = 0x1 << 3,     // 01 << 3: +/- 500 dps
        G_SCALE_1000DPS = 0x2 << 3,    // 10 << 3: +/- 1000 dps
        G_SCALE_2000DPS = 0x3 << 3     // 11 << 3: +/- 2000 dps
    };

    /// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
    enum gyro_odr
    {                               // ODR (Hz) --- Cutoff
        G_POWER_DOWN     = 0x00,    //  0           0
        G_ODR_13_BW_0    = 0x10,    //  12.5        0.0081      low power
        G_ODR_26_BW_2    = 0x20,    //  26          2.07        low power
        G_ODR_52_BW_16   = 0x30,    //  52          16.32       low power
        G_ODR_104        = 0x40,    //  104         
        G_ODR_208        = 0x50,    //  208         
        G_ODR_416        = 0x60,    //  416         
        G_ODR_833        = 0x70,    //  833         
        G_ODR_1660       = 0x80     //  1660
    };

    /// accel_scale defines all possible FSR's of the accelerometer:
    enum accel_scale
    {
        A_SCALE_2G, // 00: +/- 2g
        A_SCALE_16G,// 01: +/- 16g
        A_SCALE_4G, // 10: +/- 4g
        A_SCALE_8G  // 11: +/- 8g
    };

    /// accel_oder defines all possible output data rates of the accelerometer:
    enum accel_odr
    {
        A_POWER_DOWN,   // Power-down mode (0x0)
        A_ODR_13,       // 12.5 Hz (0x1)        low power
        A_ODR_26,       // 26 Hz (0x2)          low power
        A_ODR_52,       // 52 Hz (0x3)          low power
        A_ODR_104,      // 104 Hz (0x4)         normal mode
        A_ODR_208,      // 208 Hz (0x5)         normal mode
        A_ODR_416,      // 416 Hz (0x6)         high performance
        A_ODR_833,      // 833 Hz (0x7)         high performance
        A_ODR_1660,     // 1.66 kHz (0x8)       high performance
        A_ODR_3330,     // 3.33 kHz (0x9)       high performance
        A_ODR_6660,     // 6.66 kHz (0xA)       high performance
    };

    // accel_bw defines all possible bandwiths for low-pass filter of the accelerometer:
    enum accel_bw
    {
        A_BW_AUTO_SCALE = 0x0,  // Automatic BW scaling (0x0)
        A_BW_408 = 0x4,         // 408 Hz (0x4)
        A_BW_211 = 0x5,         // 211 Hz (0x5)
        A_BW_105 = 0x6,         // 105 Hz (0x6)
        A_BW_50 = 0x7           // 50 Hz (0x7)
    };
    
    

    // We'll store the gyro, and accel, readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call readGyro(), and readAccel() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
    int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
    int16_t temperature_raw;

    // floating-point values of scaled data in real-world units
    float gx, gy, gz;
    float ax, ay, az;
    float temperature_c, temperature_f; // temperature in celcius and fahrenheit
    float intr;

    
    /**  LSM6DS3 -- LSM6DS3 class constructor
    *  The constructor will set up a handful of private variables, and set the
    *  communication mode as well.
    *  Input:
    *   - interface = Either MODE_SPI or MODE_I2C, whichever you're using
    *               to talk to the IC.
    *   - xgAddr = If MODE_I2C, this is the I2C address of the accel/gyro.
    *               If MODE_SPI, this is the chip select pin of the accel/gyro (CS_A/G)
    */
    LSM6DS3(PinName sda, PinName scl, uint8_t xgAddr = LSM6DS3_AG_I2C_ADDR(1));
    
    /**  begin() -- Initialize the gyro, and accelerometer.
    *  This will set up the scale and output rate of each sensor. It'll also
    *  "turn on" every sensor and every axis of every sensor.
    *  Input:
    *   - gScl = The scale of the gyroscope. This should be a gyro_scale value.
    *   - aScl = The scale of the accelerometer. Should be a accel_scale value.
    *   - gODR = Output data rate of the gyroscope. gyro_odr value.
    *   - aODR = Output data rate of the accelerometer. accel_odr value.
    *  Output: The function will return an unsigned 16-bit value. The most-sig
    *       bytes of the output are the WHO_AM_I reading of the accel/gyro.
    *  All parameters have a defaulted value, so you can call just "begin()".
    *  Default values are FSR's of: +/- 245DPS, 4g, 2Gs; ODRs of 119 Hz for 
    *  gyro, 119 Hz for accelerometer.
    *  Use the return value of this function to verify communication.
    */
    uint16_t begin(gyro_scale gScl = G_SCALE_245DPS, 
                accel_scale aScl = A_SCALE_2G, gyro_odr gODR = G_ODR_104, 
                accel_odr aODR = A_ODR_104);
    
    /**  readGyro() -- Read the gyroscope output registers.
    *  This function will read all six gyroscope output registers.
    *  The readings are stored in the class' gx_raw, gy_raw, and gz_raw variables. Read
    *  those _after_ calling readGyro().
    */
    void readGyro();
    
    /**  readAccel() -- Read the accelerometer output registers.
    *  This function will read all six accelerometer output registers.
    *  The readings are stored in the class' ax_raw, ay_raw, and az_raw variables. Read
    *  those _after_ calling readAccel().
    */
    void readAccel();
    
    /**  readTemp() -- Read the temperature output register.
    *  This function will read two temperature output registers.
    *  The combined readings are stored in the class' temperature variables. Read
    *  those _after_ calling readTemp().
    */
    void readTemp();
    
    /** Read Interrupt **/
    void readIntr();
    
    /**  setGyroScale() -- Set the full-scale range of the gyroscope.
    *  This function can be called to set the scale of the gyroscope to 
    *  245, 500, or 2000 degrees per second.
    *  Input:
    *   - gScl = The desired gyroscope scale. Must be one of three possible
    *       values from the gyro_scale enum.
    */
    void setGyroScale(gyro_scale gScl);
    
    /**  setAccelScale() -- Set the full-scale range of the accelerometer.
    *  This function can be called to set the scale of the accelerometer to
    *  2, 4, 8, or 16 g's.
    *  Input:
    *   - aScl = The desired accelerometer scale. Must be one of five possible
    *       values from the accel_scale enum.
    */
    void setAccelScale(accel_scale aScl);
    
    /**  setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    *  Input:
    *   - gRate = The desired output rate and cutoff frequency of the gyro.
    *       Must be a value from the gyro_odr enum (check above).
    */
    void setGyroODR(gyro_odr gRate);
    
    /**  setAccelODR() -- Set the output data rate of the accelerometer
    *  Input:
    *   - aRate = The desired output rate of the accel.
    *       Must be a value from the accel_odr enum (check above).
    */
    void setAccelODR(accel_odr aRate);


private:    
    /**  xgAddress store the I2C address
    *  for each sensor.
    */
    uint8_t xgAddress;
    
    // I2C bus
    I2C i2c;

    /**  gScale, and aScale store the current scale range for each 
    *  sensor. Should be updated whenever that value changes.
    */
    gyro_scale gScale;
    accel_scale aScale;
    
    /**  gRes, and aRes store the current resolution for each sensor. 
    *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
    *  This value is calculated as (sensor scale) / (2^15).
    */
    float gRes, aRes;
    
    /**  initGyro() -- Sets up the gyroscope to begin reading.
    *  This function steps through all three gyroscope control registers.
    */
    void initGyro();
    
    /**  initAccel() -- Sets up the accelerometer to begin reading.
    *  This function steps through all accelerometer related control registers.
    */
    void initAccel();
    
    /** Setup Interrupt **/
    void initIntr();
    
    /**  calcgRes() -- Calculate the resolution of the gyroscope.
    *  This function will set the value of the gRes variable. gScale must
    *  be set prior to calling this function.
    */
    void calcgRes();
    
    /**  calcaRes() -- Calculate the resolution of the accelerometer.
    *  This function will set the value of the aRes variable. aScale must
    *  be set prior to calling this function.
    */
    void calcaRes();
};

#endif // _LSM6DS3_H //
