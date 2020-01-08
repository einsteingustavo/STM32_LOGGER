#include "LSM6DS3.h"

LSM6DS3::LSM6DS3(PinName sda, PinName scl, uint8_t xgAddr) : i2c(sda, scl)
{
    // xgAddress will store the 7-bit I2C address, if using I2C.
    xgAddress = xgAddr;
}

uint16_t LSM6DS3::begin(gyro_scale gScl, accel_scale aScl,  
                        gyro_odr gODR, accel_odr aODR)
{
    // Store the given scales in class variables. These scale variables
    // are used throughout to calculate the actual g's, DPS,and Gs's.
    gScale = gScl;
    aScale = aScl;
    
    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
    calcaRes(); // Calculate g / ADC tick, stored in aRes variable
    
    
    // To verify communication, we can read from the WHO_AM_I register of
    // each device. Store those in a variable so we can return them.
    // The start of the addresses we want to read from
    char cmd[2] = {
        WHO_AM_I_REG,
        0
    };

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(xgAddress, cmd+1, 1);
    uint8_t xgTest = cmd[1];                    // Read the accel/gyro WHO_AM_I
        
    // Gyro initialization stuff:
    initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
    setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
    setGyroScale(gScale); // Set the gyro range
    
    // Accelerometer initialization stuff:
    initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
    setAccelODR(aODR); // Set the accel data rate.
    setAccelScale(aScale); // Set the accel range.
    
    // Interrupt initialization stuff;
    initIntr();
    
    // Once everything is initialized, return the WHO_AM_I registers we read:
    return xgTest;
}

void LSM6DS3::initGyro()
{
    char cmd[4] = {
        CTRL2_G,
        (const char)(gScale | G_ODR_104),
        0,          // Default data out and int out
        0           // Default power mode and high pass settings
    };

    // Write the data to the gyro control registers
    i2c.write(xgAddress, cmd, 4);
}

void LSM6DS3::initAccel()
{
    char cmd[4] = {
        CTRL1_XL,
        0x38,       // Enable all axis and don't decimate data in out Registers
        (const char)((A_ODR_104 << 5) | (aScale << 3) | (A_BW_AUTO_SCALE)),   // 119 Hz ODR, set scale, and auto BW
        0           // Default resolution mode and filtering settings
    };

    // Write the data to the accel control registers
    i2c.write(xgAddress, cmd, 4);
}

void LSM6DS3::initIntr()
{
    char cmd[2];
    
    cmd[0] = TAP_CFG;
    cmd[1] = 0x0E;
    i2c.write(xgAddress, cmd, 2);
    cmd[0] = TAP_THS_6D;
    cmd[1] = 0x03;
    i2c.write(xgAddress, cmd, 2);
    cmd[0] = INT_DUR2;
    cmd[1] = 0x7F;
    i2c.write(xgAddress, cmd, 2);
    cmd[0] = WAKE_UP_THS;
    cmd[1] = 0x80;
    i2c.write(xgAddress, cmd, 2);
    cmd[0] = MD1_CFG;
    cmd[1] = 0x48;
    i2c.write(xgAddress, cmd, 2);
}

void LSM6DS3::readAccel()
{
    // The data we are going to read from the accel
    char data[6];

    // Set addresses
    char subAddressXL = OUTX_L_XL;
    char subAddressXH = OUTX_H_XL;
    char subAddressYL = OUTY_L_XL;
    char subAddressYH = OUTY_H_XL;
    char subAddressZL = OUTZ_L_XL;
    char subAddressZH = OUTZ_H_XL;

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, &subAddressXL, 1, true);
    // Read in register containing the axes data and alocated to the correct index
    i2c.read(xgAddress, data, 1);
    
    i2c.write(xgAddress, &subAddressXH, 1, true);
    i2c.read(xgAddress, (data + 1), 1);
    i2c.write(xgAddress, &subAddressYL, 1, true);
    i2c.read(xgAddress, (data + 2), 1);
    i2c.write(xgAddress, &subAddressYH, 1, true);
    i2c.read(xgAddress, (data + 3), 1);
    i2c.write(xgAddress, &subAddressZL, 1, true);
    i2c.read(xgAddress, (data + 4), 1);
    i2c.write(xgAddress, &subAddressZH, 1, true);
    i2c.read(xgAddress, (data + 5), 1);

    // Reassemble the data and convert to g
    ax_raw = data[0] | (data[1] << 8);
    ay_raw = data[2] | (data[3] << 8);
    az_raw = data[4] | (data[5] << 8);
    ax = ax_raw * aRes;
    ay = ay_raw * aRes;
    az = az_raw * aRes;
}

void LSM6DS3::readIntr()
{
    char data[1];
    char subAddress = TAP_SRC;

    i2c.write(xgAddress, &subAddress, 1, true);
    i2c.read(xgAddress, data, 1);

    intr = (float)data[0];
}

void LSM6DS3::readTemp()
{
    // The data we are going to read from the temp
    char data[2];

    // Set addresses
    char subAddressL = OUT_TEMP_L;
    char subAddressH = OUT_TEMP_H;

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, &subAddressL, 1, true);
    // Read in register containing the temperature data and alocated to the correct index
    i2c.read(xgAddress, data, 1);
    
    i2c.write(xgAddress, &subAddressH, 1, true);
    i2c.read(xgAddress, (data + 1), 1);

    // Temperature is a 12-bit signed integer   
    temperature_raw = data[0] | (data[1] << 8);

    temperature_c = (float)temperature_raw / 16.0 + 25.0;
    temperature_f = temperature_c * 1.8 + 32.0;
}


void LSM6DS3::readGyro()
{
    // The data we are going to read from the gyro
    char data[6];

    // Set addresses
    char subAddressXL = OUTX_L_G;
    char subAddressXH = OUTX_H_G;
    char subAddressYL = OUTY_L_G;
    char subAddressYH = OUTY_H_G;
    char subAddressZL = OUTZ_L_G;
    char subAddressZH = OUTZ_H_G;

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, &subAddressXL, 1, true);
    // Read in register containing the axes data and alocated to the correct index
    i2c.read(xgAddress, data, 1);
    
    i2c.write(xgAddress, &subAddressXH, 1, true);
    i2c.read(xgAddress, (data + 1), 1);
    i2c.write(xgAddress, &subAddressYL, 1, true);
    i2c.read(xgAddress, (data + 2), 1);
    i2c.write(xgAddress, &subAddressYH, 1, true);
    i2c.read(xgAddress, (data + 3), 1);
    i2c.write(xgAddress, &subAddressZL, 1, true);
    i2c.read(xgAddress, (data + 4), 1);
    i2c.write(xgAddress, &subAddressZH, 1, true);
    i2c.read(xgAddress, (data + 5), 1);

    // Reassemble the data and convert to degrees/sec
    gx_raw = data[0] | (data[1] << 8);
    gy_raw = data[2] | (data[3] << 8);
    gz_raw = data[4] | (data[5] << 8);
    gx = gx_raw * gRes;
    gy = gy_raw * gRes;
    gz = gz_raw * gRes;
}

void LSM6DS3::setGyroScale(gyro_scale gScl)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        CTRL2_G,
        0
    };

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(xgAddress, cmd+1, 1);

    // Then mask out the gyro scale bits:
    cmd[1] &= 0xFF^(0x3 << 3);
    // Then shift in our new scale bits:
    cmd[1] |= gScl << 3;

    // Write the gyroscale out to the gyro
    i2c.write(xgAddress, cmd, 2);
    
    // We've updated the sensor, but we also need to update our class variables
    // First update gScale:
    gScale = gScl;
    // Then calculate a new gRes, which relies on gScale being set correctly:
    calcgRes();
}

void LSM6DS3::setAccelScale(accel_scale aScl)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        CTRL1_XL,
        0
    };

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(xgAddress, cmd+1, 1);

    // Then mask out the accel scale bits:
    cmd[1] &= 0xFF^(0x3 << 3);
    // Then shift in our new scale bits:
    cmd[1] |= aScl << 3;

    // Write the accelscale out to the accel
    i2c.write(xgAddress, cmd, 2);
    
    // We've updated the sensor, but we also need to update our class variables
    // First update aScale:
    aScale = aScl;
    // Then calculate a new aRes, which relies on aScale being set correctly:
    calcaRes();
}

void LSM6DS3::setGyroODR(gyro_odr gRate)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        CTRL2_G,
        0
    };
    
    // Set low power based on ODR, else keep sensor on high performance
    if(gRate == G_ODR_13_BW_0 | gRate == G_ODR_26_BW_2 | gRate == G_ODR_52_BW_16) {
        char cmdLow[2] ={
            CTRL7_G,
            1
        };
        
        i2c.write(xgAddress, cmdLow, 2);
    }
    else {
        char cmdLow[2] ={
            CTRL7_G,
            0
        };
        
        i2c.write(xgAddress, cmdLow, 2);
    }

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(xgAddress, cmd+1, 1);

    // Then mask out the gyro odr bits:
    cmd[1] &= (0x3 << 3);
    // Then shift in our new odr bits:
    cmd[1] |= gRate;

    // Write the gyroodr out to the gyro
    i2c.write(xgAddress, cmd, 2);
}

void LSM6DS3::setAccelODR(accel_odr aRate)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        CTRL1_XL,
        0
    };
    
    // Set low power based on ODR, else keep sensor on high performance
    if(aRate == A_ODR_13 | aRate == A_ODR_26 | aRate == A_ODR_52) {
        char cmdLow[2] ={
            CTRL6_C,
            1
        };
        
        i2c.write(xgAddress, cmdLow, 2);
    }
    else {
        char cmdLow[2] ={
            CTRL6_C,
            0
        };
        
        i2c.write(xgAddress, cmdLow, 2);
    }

    // Write the address we are going to read from and don't end the transaction
    i2c.write(xgAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(xgAddress, cmd+1, 1);

    // Then mask out the accel odr bits:
    cmd[1] &= 0xFF^(0x7 << 5);
    // Then shift in our new odr bits:
    cmd[1] |= aRate << 5;

    // Write the accelodr out to the accel
    i2c.write(xgAddress, cmd, 2);
}

void LSM6DS3::calcgRes()
{
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10).
    switch (gScale)
    {
        case G_SCALE_245DPS:
            gRes = 245.0 / 32768.0;
            break;
        case G_SCALE_500DPS:
            gRes = 500.0 / 32768.0;
            break;
        case G_SCALE_2000DPS:
            gRes = 2000.0 / 32768.0;
            break;
    }
}

void LSM6DS3::calcaRes()
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100).
    switch (aScale)
    {
        case A_SCALE_2G:
            aRes = 2.0 / 32768.0;
            break;
        case A_SCALE_4G:
            aRes = 4.0 / 32768.0;
            break;
        case A_SCALE_8G:
            aRes = 8.0 / 32768.0;
            break;
        case A_SCALE_16G:
            aRes = 16.0 / 32768.0;
            break;
    }
}