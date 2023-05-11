#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <string.h>
using namespace std;

// Define the I2C address of the Arduino Nano
#define ARDUINO_ADDR 0x08

class I2C
{
private:
    int file;
    char filename[20];
    uint8_t data[3];

public:
    // constructor
    int I2C()
    {
        SetupConnection();
    };

    int SetupConnection()
    {
        // returns 0 if failed

        // Open the I2C bus on the Raspberry Pi
        sprintf(filename, "/dev/i2c-1");
        if ((file = open(filename, O_RDWR)) < 0)
        {
            printf("Failed to open the I2C bus\n");
            return 0;
        }

        // Set the I2C slave address of the Arduino Nano
        if (ioctl(file, I2C_SLAVE, ARDUINO_ADDR) < 0)
        {
            printf("Failed to set the I2C slave address\n");
            close(file);
            return 0;
        }
        return 1;
    }
    int SendData(int ServoAngle, int Speed)
    {
        data[0] = ServoAngle;
        data[1] = Speed;
        data[2] = Speed;

        data[0] = ServoAngle;
        if (write(file, data, 3) != 3)
        {
            printf("Failed to write to the I2C bus\n");
            close(file);
            return 0;
        }
        printf("Servo angle set to %d, motor speeds set to %hhu, %hhu\n", ServoAngle, data[1], data[2]);
        return 1;
    }
    int CloseConnection()
    {
        close(file);
        return 1;
    }
};