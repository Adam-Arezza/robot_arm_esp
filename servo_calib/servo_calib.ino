#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct ServoMotor {
  int pin;
  int minpulse;
  int maxpulse;
  int defaultPosition;
  int currentPosition;
  int targetPosition;
  int potVal;
  int potMax;
  int potMin;
  const int potPin;
};


//array of servos for each joint
ServoMotor armServos[5] = {
  { 0, 420, 135, 180, 0, 0, 0, 4000, 800, 14 }, 
  { 1, 230, 490, 90, 0, 0, 0, 450, 3350, 27 },  
  { 2, 100, 490, 90, 0, 0, 0, 3250, 420, 26 },  
  { 3, 120, 510, 90, 0, 0, 0, 3200, 375, 25 },  
  { 4, 200, 400, 0, 0, 0, 0, 0, 0, 0 },         
};

void setup() {

  //start serial
  Serial.begin(115200);
  Serial.setTimeout(50);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  //start-up delay
  delay(50);
}

//change the servo number to sweep 
void loop (){
  for(int i = armServos[1].minpulse; i < armServos[1].maxpulse; i++){
      pwm.setPWM(armServos[1].pin, 0, i);
      delay(20);
  }
  Serial.println("At max pulse.");
  delay(3000);

  for(int i = armServos[1].maxpulse; i > armServos[1].minpulse; i--){
      pwm.setPWM(armServos[1].pin, 0, i);
      delay(20);
  }
  Serial.println("At min pulse.");
  delay(3000);
}
