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

struct RotaryEncoder {
  int pin_A;
  int pin_B;
  int sw;
  int currentState;
  int lastState;
  volatile int count;
};

int joint_selected = 0;

//array of servos for each joint
ServoMotor armServos[5] = {
  { 0, 100, 700, 0, 0, 0, 0, 14 },  //100 - 500 = 0 - 180
  { 1, 180, 440, 0, 0, 0, 0, 27 },  //180 - 440 = 0 - 180
  { 2, 125, 500, 0, 0, 0, 0, 26 },  //125 - 500 = 0 - 180
  { 3, 110, 470, 0, 0, 0, 0, 25 },  //110 - 470 = 0 - 180
  { 4, 200, 400, 0, 0, 0, 0, 0 },         //315 is full close, 420 is full open
};

//encoder for manual control
RotaryEncoder encoder = { 35, 34, 32, 0, 0, 0 };

//counter for encoder pulses
//void IRAM_ATTR counter() {
//  encoder.count += digitalRead(encoder.pin_B) == HIGH ? 1 : -1;
//  encoder.currentState = encoder.count;
//}

String prev_response = "";

void setup() {

  //start serial
  Serial.begin(115200);
  Serial.setTimeout(50);

  //start pwm driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  //set pins for encoder
  pinMode(encoder.pin_A, INPUT);
  pinMode(encoder.pin_B, INPUT);
  pinMode(encoder.sw, INPUT);

  //start-up delay
  delay(50);
}

// Function to set the position of a servo
// void setServoPosition(int servoIndex) {
//   if (servoIndex >= 0 && servoIndex < 5) {
//     int current_degrees = armServos[servoIndex].currentPosition;  // in pulsewidth
//     int degrees = armServos[servoIndex].targetPosition;           // in degrees
//     degrees = constrain(degrees, 0, 180);
//     int pulseWidth = map(degrees, 0, 180, armServos[servoIndex].minpulse, armServos[servoIndex].maxpulse);  // convert the target to pulsewidth
//     int pulse_dir = (pulseWidth > armServos[servoIndex].currentPosition) ? 1 : -1;                          // get direction
//     int pulse = current_degrees + (increment * pulse_dir);
//     pulse = constrain(pulse, armServos[servoIndex].minpulse, armServos[servoIndex].maxpulse);
//     pwm.setPWM(armServos[servoIndex].pin, 0, pulse);  // increment/decrement the pulsewidth by the increment variable
//     armServos[servoIndex].currentPosition = pulse;    // Update the current position to the incremented value
//     delay(delay_time);
//   }
// }

void set_target_positions(String &s){
    char start_char = '<';
    char delimiter = ':';
    int number_of_joints = 0;
    String temp_str = "";
    for (int i = 0; i < s.length() + 1; i++) {
        if (s[i] == start_char) {
            continue;
        }
        if (s[i] == delimiter || i == s.length()) {
            armServos[number_of_joints].targetPosition = temp_str.toInt();
            number_of_joints += 1;
            temp_str = "";
        } else {
            temp_str += s[i];
        }
    }
}


void send_online_msg() {
    //get all the pot values and send the data
    String online_response = "";
    for(int i = 0; i < sizeof(armServos) / sizeof(armServos[0]); i++) {
        //online_response += armServos[i].potVal;
        online_response += armServos[i].currentPosition;
        //need to remove the extra ':' at the end
        if (i == sizeof(armServos) / sizeof(armServos[0]) - 1) {
            break;
        }
        online_response += ":";
    }
    Serial.print(online_response);
}


void parser(String s) {
    //check string has values
    if (s.length() >= 1) {
        if (s.startsWith("online", 1)){
            send_online_msg();
            return;
        }

        else{
            //need more validation here, check if the format is correct for sending joint target angles
            set_target_positions(s);
        }
    }
}


void move_servo(ServoMotor &servo) {
  int dir = 1;
  if (servo.currentPosition > servo.targetPosition) {
    dir = -1;
  }
  int diff = abs(servo.currentPosition - servo.targetPosition);
  if (diff > 0) {
    servo.currentPosition += 1 * dir;
    //move the servo motor
  }
}


void loop() {
  String response = "";

  //set each of the servos current pot values
  for (int i = 0; i < 5; i++) {
    int mapped_pot_val = map(analogRead(armServos[i].potPin), 0, 4096, 0, 180);
    armServos[i].potVal = mapped_pot_val;
  }

  //Read in commands and parse the data
  if (Serial.available() > 0) {
    String command_data = Serial.readString();
    command_data.trim();
    if(command_data.startsWith("<") && command_data.endsWith(">")){
        parser(command_data);
    }
    Serial.flush();
  }

  //if a servo is not at it's target, increment it towards it's target
  for (int s = 0; s < sizeof(armServos) / sizeof(armServos[0]); s++) {
    if (armServos[s].targetPosition != armServos[s].currentPosition) {
      move_servo(armServos[s]);
    }
    response += String(armServos[s].currentPosition);
    if (s == sizeof(armServos) / sizeof(armServos[0]) - 1) {
      break;
    } else {
      response += ":";
    }
  }

  if (response != prev_response){
      if (response.length() > 0 && response.begin() != ":") {
          Serial.println(response);
          prev_response = response;
      }
  }
  Serial.flush();
  delay(25);
}
