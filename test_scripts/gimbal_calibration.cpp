#include <Arduino.h>
#include <Servo.h>

Servo pitchServo;
Servo yawServo;

void setup() {
    pitchServo.attach(9);
    yawServo.attach(10);

    pitchServo.writeMicroseconds(1500); 
    yawServo.writeMicroseconds(1500);

    delay(2000);

}

void loop() {
// start slightly to the left, and increment to the right
    for (int pos = 1350; pos < 1650; pos += 5) {
        pitchServo.writeMicroseconds(pos);

    delay(15);
    }

        for (int pos = 1650; pos >= 1350; pos -= 5) {
            pitchServo.writeMicroseconds(pos);
        
            delay(15);
        }

    pitchServo.writeMicroseconds(1500);
    
    delay(1000);
}