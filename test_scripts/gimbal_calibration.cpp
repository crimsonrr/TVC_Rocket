#include <Arduino.h>
#include <Servo.h>

Servo pitchServo; // pin 10
Servo yawServo;   // pin 9

// can be tuned
int yawCenter = 1700;  
int yawRange  = 150;    // ~30° each side

int pitchCenter = 1500;
int pitchRange  = 200;

void setup() {
    pitchServo.attach(10);
    yawServo.attach(9);

    pitchServo.writeMicroseconds(pitchCenter);
    yawServo.writeMicroseconds(yawCenter);

    delay(2000);
}

void loop() {

    // yaw servo
    for (int pos = yawCenter - yawRange; pos <= yawCenter + yawRange; pos += 5) {
        yawServo.writeMicroseconds(pos);
        delay(10);
    }

    for (int pos = yawCenter + yawRange; pos >= yawCenter - yawRange; pos -= 5) {
        yawServo.writeMicroseconds(pos);
        delay(10);
    }

    // return back to the center
    yawServo.writeMicroseconds(yawCenter);

    delay(500);

    // pitch
    for (int pos = pitchCenter - pitchRange; pos <= pitchCenter + pitchRange; pos += 5) {
        pitchServo.writeMicroseconds(pos);
        delay(10);
    }

    for (int pos = pitchCenter + pitchRange; pos >= pitchCenter - pitchRange; pos -= 5) {
        pitchServo.writeMicroseconds(pos);
        delay(10);
    }

    pitchServo.writeMicroseconds(pitchCenter);

    delay(500);
}