#ifndef I2C_HPP
#define I2C_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <string.h>

class I2C
{
public:
    // constructor
    int I2C();

    int SetupConnection();
    int SendData(int ServoAngle, int Speed);
    int CloseConnection();

private:
    int file;
    char filename[20];
    uint8_t data[3];
};

#endif