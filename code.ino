#include <SoftwareSerial.h> 
#include <Servo.h>

#define enA 10 // Attach enA to 10 
#define in1 4 // Attach in1 to 4 
#define in2 5 // Attach in2 to 5 
#define enB 11 // Attach enB to 11 
#define in3 6 // Attach in3 to 6 
#define in4 7 // Attach in4 to 7 
#define enC 12 // Attach enC to 12 
#define in5 8 // Attach in5 to 8 
#define in6 9 // Attach in6 to 9

Servo servo01; 
Servo servo02;

SoftwareSerial Bluetooth(0,1); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

int servo1Pos, servo2Pos; // current position
int servo1PPos, servo2PPos; // previous position 
String dataIn = "";
int xAxis, yAxis;
int x=0;
int y=0;
int angle = 0;
int motorSpeedA = 0;
int motorSpeedB = 0;

void setup() {
    servo01.attach(3); // Attach wrist servo signal to 22 
    servo02.attach(13); // Attach grip servo signal to 24 
    Bluetooth.begin(9600); // Default baud rate of the Bluetooth module Bluetooth.setTimeout(1);
    delay(20);
// Robot arm initial position servo1PPos = 85; servo01.write(servo1PPos); servo2PPos = 80; servo02.write(servo2PPos);
    pinMode(enA, OUTPUT); 
    pinMode(enB, OUTPUT); 
    pinMode(enC, OUTPUT); 
    pinMode(in1, OUTPUT); 
    pinMode(in2, OUTPUT); 
    pinMode(in3, OUTPUT); 
    pinMode(in4, OUTPUT);
    pinMode(in5, OUTPUT);
    pinMode(in6, OUTPUT);
}

void loop() {
    // Check for incoming data
    if (Bluetooth.available() > 0) {
        dataIn = Bluetooth.readString(); // Read the data as string 
        if(dataIn.equals("xymotion")) {
            x = Bluetooth.read(); 
            delay(10);
            y = Bluetooth.read(); 
            xyMotion(x,y);
        }
        if(dataIn.startsWith("wrist")) {
            servo1Pos = dataIn.substring(5, dataIn.length()).toInt(); // Reads the data from the serial port
            wrist(servo1Pos);
        }
        if(dataIn.startsWith("grip")) {
            angle = dataIn.substring(4, dataIn.length()).toInt(); // Reads the data from the serial port 
            grip(angle);
        }
        if(dataIn.equals("move up")) {
            // Set Z-axis Motor Up
            digitalWrite(in5, LOW);
            digitalWrite(in6, HIGH);
            analogWrite(enC, 255); // Send PWM signal to motor C
        }
        if(dataIn.equals("move down")) {
            // Set Z-axis Motor Down
            digitalWrite(in5, HIGH);
            digitalWrite(in6, LOW);
            analogWrite(enC, 255); // Send PWM signal to motor C
        }
        if(dataIn.equals("stop")) {
            // Stop Z-axis Motor
            digitalWrite(in6, LOW);
            digitalWrite(in5, LOW);
            analogWrite(enC, 0); // Send PWM signal to motor C
        }
    }
    delayMicroseconds(100); 
}

void xyMotion(int x,int y) {
    // Default value - no movement when the Joystick stays in the center 
    int xAxis = 510;
    int yAxis = 510;
   
    // Makes sure we receive corrent values
    if (x > 60 & x < 220) {
        xAxis = map(x, 220, 60, 1023, 0); // Convert the smartphone X and Y values to 0 - 1023 range, suitable motor for the motor control code below
    }
    if (y > 60 & y < 220) {
        yAxis = map(y, 220, 60, 0, 1023);
    } 

    // Y-axis used for forward and backward control 
    if (yAxis < 470) {
        // Set Motor A backward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        // Set Motor B backward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255value for the PWM signal for increasing the motor speed 
        motorSpeedA = map(yAxis, 470, 0, 0, 255); 
        motorSpeedB = map(yAxis, 470, 0, 0, 255);
    }
    else if (yAxis > 550) {
        // Set Motor A forward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        // Set Motor B forward
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to255 value for the PWM signal for increasing the motor speed 
        motorSpeedA = map(yAxis, 550, 1023, 0, 255); 
        motorSpeedB = map(yAxis, 550, 1023, 0, 255);
    }
    // If joystick stays in middle the motors are not moving 
    else {
        motorSpeedA = 0;
        motorSpeedB = 0;
    }
    // X-axis used for left and right control 
    if (xAxis < 470) {
        // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value 
        int xMapped = map(xAxis, 470, 0, 0, 255);
        // Move to left - decrease left motor speed, increase right motor speed 
        motorSpeedA = motorSpeedA - xMapped;
        motorSpeedB = motorSpeedB + xMapped; 
        // Confine the range from 0 to 255
        if (motorSpeedA < 0) {
            motorSpeedA = 0;
        }
        if (motorSpeedB > 255) { 
            motorSpeedB = 255;
        }
    }
    if (xAxis > 550) {
        // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value 
        int xMapped = map(xAxis, 550, 1023, 0, 255);
        // Move right - decrease right motor speed, increase left motor speed 
        motorSpeedA = motorSpeedA + xMapped;
        motorSpeedB = motorSpeedB - xMapped;
        // Confine the range from 0 to 255
        if (motorSpeedA > 255) {
            motorSpeedA = 255;
        }
        if (motorSpeedB < 0) { 
            motorSpeedB = 0;
        }
    }
    
    // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
    if (motorSpeedA < 70) { 
        motorSpeedA = 0;
    }
    if (motorSpeedB < 70) {
        motorSpeedB = 0; 
    }
    analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
    analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}

void wrist(int servo1Pos) {
    if (servo1PPos > servo1Pos) {
        for ( int j = servo1PPos; j >= servo1Pos; j--) { 
            servo01.write(j);
            delay(30);
        }
    }
    if (servo1PPos < servo1Pos) {
        for ( int j = servo1PPos; j <= servo1Pos; j++) {
            servo01.write(j);
            delay(30);
        }
    }
    servo1PPos = servo1Pos;
}

void grip(int servo2Pos) {
    if (servo2PPos > servo2Pos) {
        for ( int j = servo2PPos; j >= servo2Pos; j--) { 
            servo02.write(j);
            delay(30);
        }
    }
    if (servo2PPos < servo2Pos) {
        for ( int j = servo2PPos; j <= servo2Pos; j++) { 
            servo02.write(j);
            delay(30);
        }
    }
    servo2PPos = servo2Pos;
}
