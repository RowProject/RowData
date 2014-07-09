#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define BAUDRATE 9600

typedef MPU6050 MPU9150 // get yer type names right, christ

MPU9150 mpu(0x68); // Find the MPU on the I2C network at address 0x68

unsigned long time;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// Buffer array to be sent to the computer
uint8_t statePacket[10] = {
	'$'			// Indicates a state packet
	0, 0, 0,	// Accelermeter
	0, 0, 0,	// Gyroscope
	0, 0, 0		// Magnetometer
};

void setup() {
    Serial.begin(BAUDRATE);
	Serial.println("Arduino data thing started");
    Wire.begin(); // Join I2C bus
    Serial.println("Initializing I2C devices...");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); // verify connection
    Serial.println("");
}

void loop() {
    // Read the raw inputs (although we won't really need the magnetometer)
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    statePacket[1] = ax;
    statePacket[2] = ay;
    statePacket[3] = az;
    statePacket[4] = gx;
    statePacket[5] = gy;
    statePacket[6] = gz;
    statePacket[7] = mx;
    statePacket[8] = my;
    statePacket[9] = mz;
    Serial.write(statePacket, 10);
}