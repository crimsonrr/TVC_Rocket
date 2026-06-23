#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo pitchServo;
Servo yawServo;

int16_t ax, ay, az;

void setup() {
    Wire.begin();
    Serial.begin(9600); // start serial communication 

    Serial.println("Initializing MPU6050...");
    mpu.initialize(); // wake up the sensor

    if (mpu.testConnection()) { 
        Serial.println("MPU6050 connection successful.");
    } else {
        Serial.println("MPU6050 connection failed.");
    }

    Wire.beginTransmission(0x68); // MPU6050 I2C address
    Wire.write(0x6B); // where 0x68 is the I2C address of the MPU6050 
    Wire.write(0); // wake up the MPU6050
    Wire.endTransmission(true);
}

void loop() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // (ACCEL_XOUT_H) register
    Wire.endTransmission(false);

    Wire.requestFrom(0x68, 6, true); // request 6 (for each axis) bytes from the MPU6050 

// comnbine the low and high bytes for each axis to get the full 16-bit value
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

// print the values to the serial monitor
    Serial.print("aX = "); Serial.print(ax); 
    Serial.print("aY = "); Serial.print(ay); 
    Serial.print("aZ = "); Serial.println(az);

    delay(1000);

}