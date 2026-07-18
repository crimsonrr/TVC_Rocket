#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <cmath> 

MPU6050 mpu;
Servo pitchServo;
Servo rollServo;

int16_t ax, ay, az; // accelerometer values
int16_t gx, gy, gz; // gyroscope values
int16_t gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0; // account for sensor drift

unsigned long previousTime = 0;

// PD controller variables
float kp = 2.0; // proportional gain for pitch
float kd = 0.4; // derivative gain for pitch
float alpha = 0.98; // filter coefficient

float pitchEstimate = 0; 
float rollEstimate = 0;
float targetpitch = 0; 
float targetroll = 0;

void setup() {
    Wire.begin();
    Serial.begin(115200); // start serial communication

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

    calibrateGyro();
    Serial.println("Gyroscope calibrated.");

    pitchServo.attach(10); 
    rollServo.attach(9);

    pitchServo.writeMicroseconds(1500);
    rollServo.writeMicroseconds(1500);

    previousTime = millis(); // return how much time the setup took
}

void calibrateGyro() {
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0; // start with zero before reading values for consistency
    int samples = 100; // number of samples to take for calibration

    for (int i = 0; i < samples; i++) {
       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // read the accelerometer and gyroscope values
        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;
        
        delay(10); // space out each 1000 samples by 3 ms

    }

// take the error of all 1000 readings and divide by the number of samples to get the average error for each axis
    gyroXOffset = gyroXSum / samples;
    gyroYOffset = gyroYSum / samples;
    gyroZOffset = gyroZSum / samples;

    Serial.print("gX = "); Serial.print(gyroXSum); 
    Serial.print("gY = "); Serial.print(gyroYSum); 
    Serial.print("gZ = "); Serial.println(gyroZSum);

    Serial.print("Gyroscope offsets: X = "); Serial.print(gyroXOffset);
    Serial.print("Gyroscope offsets: Y = "); Serial.print(gyroYOffset);
    Serial.print("Gyroscope offsets: Z = "); Serial.println(gyroZOffset);
}

float getPitch() {
    // calulate the pitch angle using the accelerometer values
    float pitch = atan2(-ax, sqrt((ay * ay) + (az * az))) * 180.0 / PI;

    return pitch;
}

float getRoll() {
    // calulate the roll angle using the accelerometer values
    float roll = atan2(ay, sqrt((ax * ax) + (az * az))) * 180.0 / PI;
   
    return roll;
}

void loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // read the accelerometer and gyroscope values

// subtract the offset from the current reading to get the corrected value
    gx -= gyroXOffset;
    gy -= gyroYOffset; 
    gz -= gyroZOffset; 
   
    float pitch = getPitch(); 
    float roll = getRoll(); 
    
    Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print(" degrees, ");
    Serial.print("Roll: "); Serial.print(roll, 2); Serial.println("degrees");

// 131 counts = 1 deg/s     
    float gyroPitchRate = gx / 131.0;
    float gyroRollRate = gy / 131.0;

    Serial.print("Gyro Pitch Rate: "); Serial.print(gyroPitchRate, 2); Serial.print(" deg/s, ");
    Serial.print("Gyro Roll Rate: "); Serial.print(gyroRollRate, 2); Serial.println(" deg/s");

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0; // convert to seconds
    if (dt <= 0 || dt > 0.1) dt = 0.01; 
    previousTime = currentTime;

// essentially euler integration, where theta = a(gyro estimate) + (1-a)(accel estimate), where a is the filter coefficient
// estimates are computed by integrating the gyroscope rate over time and adding it to the previous estimate, while the accelerometer provides an absolute reference for the angle
    pitchEstimate = alpha * (pitchEstimate + gyroPitchRate * dt) + (1 - alpha) * pitch; // integrate gyroscope rate to estimate pitch
    rollEstimate = alpha * (rollEstimate + gyroRollRate * dt) + (1 - alpha) * roll; // integrate gyroscope rate to estimate roll

    float pitchError = targetpitch - pitchEstimate; // desired pitch is 0 degrees
    float rollError = targetroll - rollEstimate; 

    // PD control (derivative should oppose motion, hence the negative sign)
    float pitchOutput = (kp * pitchError) - (kd * gyroPitchRate);
    float rollOutput = (kp * rollError) - (kd * gyroRollRate);

    int pitchServoPosition = 1500 + pitchOutput * 5; // 1500 is the center position for the servo
    int rollServoPosition = 1500 + rollOutput * 5; // multiply by 5 to scale the output 

    pitchServoPosition = constrain(pitchServoPosition, 1300, 1700);
    rollServoPosition  = constrain(rollServoPosition, 1300, 1700);

    Serial.print("Pitch Servo: ");
    Serial.print(pitchServoPosition);

    Serial.print("      Roll Servo: ");
    Serial.print(rollServoPosition);

    pitchServo.writeMicroseconds(pitchServoPosition);
    rollServo.writeMicroseconds(rollServoPosition);

    if (abs(pitchOutput) < 0.2) {
        pitchOutput = 0;
    }

    if (abs(rollOutput) < 0.2) {
        rollOutput = 0;
    }
}