#include <QTRSensors.h>
#include <AFMotor.h>

#define NUM_SENSORS   5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

AF_DCMotor motorLeft(1); // Connect the left motor to M1
AF_DCMotor motorRight(2); // Connect the right motor to M2

// PD Controller parameters
float Kp = 0.2; // Proportional constant
float Kd = 0.4; // Derivative constant
//float Ki = 0.0015; // Integral constant

unsigned int lastError = 0; // For storing the last error
//unsigned int integral = 0; // For storing the integral of the error

// Threshold for Integral Wind-Up
//unsigned int integralThreshold = 5000;


void setup()
{
  delay(1000);

  motorLeft.setSpeed(95);
  motorRight.setSpeed(95);

  //Auto calibrate in 4.15s
  unsigned long startTime = millis();
  while(millis() - startTime < 4150)
  {
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
    qtrrc.calibrate();
  }

  motorLeft.run(RELEASE);
  motorRight.run(RELEASE); 

  delay(2000);
}

void loop()
{
  unsigned int position = qtrrc.readLine(sensorValues);
  
  // Calculate the deviation from the middle of the line (1600)
//  int error = position - 1600;
  //For white line
  int error = 1600 - position;

  
  // Calculate the integral of the error
//  integral += error;

//  // Implement Integral Wind-Up limit
//  if (integral > integralThreshold)
//    integral = integralThreshold;
//  if (integral < -integralThreshold)
//    integral = -integralThreshold;

  // Calculate the derivative of the error
  int derivative = error - lastError;
  
  // PID Controller: Compute the control variable
  int control = Kp * error + Kd * derivative;

  // Save the current error as last error for the next loop iteration
  lastError = error;

  // Adjust motor speeds
  int leftSpeed = 95 + control;
  int rightSpeed = 95 - control;

  // Ensure motor speeds are within bounds [0,127]
  leftSpeed = constrain(leftSpeed, 0, 127);
  rightSpeed = constrain(rightSpeed, 0, 127);
 
  // Set motor speeds and direction
  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(rightSpeed);

  if (control > 0) {
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
  } else {
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
  }
  
  //For white line
//    if (control < 0) {
//    motorLeft.run(FORWARD);
//    motorRight.run(BACKWARD);
//  } else {
//    motorLeft.run(BACKWARD);
//    motorRight.run(FORWARD);
//  }

}
