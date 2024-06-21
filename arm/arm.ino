#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int delay_time = 50;

struct ServoMotor 
{
  int pin;
  int minpulse;
  int maxpulse;
  int defaultPosition;
  int currentPosition;
  int targetPosition;
  int minPosition;
  int maxPosition;
  int potVal;
  int potMax;
  int potMin;
  const int potPin;
};

struct RotaryEncoder 
{
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
  { 0, 420, 135, 180, 0, 0, 0, 180, 0, 4000, 800, 14 }, 
  { 1, 230, 490, 90, 0, 0, 0, 180, 0, 3350, 450, 27 },  
  { 2, 100, 490, 0, 0, 0,-90, 90, 0, 3250, 420, 26 },  
  { 3, 120, 490, 0, 0, 0,-90, 90, 0, 3200, 375, 25 },  
  { 4, 200, 400, 0, 0, 0, 0, 180, 0, 0, 0, 0 },         
};

String prev_response = "";

void setup() {

  //start serial
  Serial.begin(115200);
  Serial.setTimeout(50);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  startup();

  //start-up delay
  delay(50);
}

void startup() {
  // get the current position of each servo and set it's pwm value to the value it's currently at
  for (int i = 0; i < 5; i++)
  {
    armServos[i].targetPosition = armServos[i].defaultPosition;
    int current_pos = analogRead(armServos[i].potPin);

    current_pos = map(current_pos, 
                      armServos[i].potMin, 
                      armServos[i].potMax, 
                      armServos[i].minPosition, 
                      armServos[i].maxPosition);

    armServos[i].currentPosition = current_pos;

    int pulse = map(current_pos, 
                    armServos[i].minPosition, 
                    armServos[i].maxPosition, 
                    armServos[i].minpulse, 
                    armServos[i].maxpulse);

    pwm.setPWM(armServos[i].pin, 0, pulse);
  }
  delay(1000);
}


void set_target_positions(String &s){
    char start_char = '<';
    char delimiter = ':';
    int number_of_joints = 0;
    String temp_str = "";

    for (int i = 0; i < s.length() + 1; i++) 
    {

        if (s[i] == start_char) 
        {
            continue;
        }

        if (s[i] == delimiter || i == s.length()) 
        {
            armServos[number_of_joints].targetPosition = temp_str.toInt();
            number_of_joints += 1;
            temp_str = "";
        } 

        else 
        {
            temp_str += s[i];
        }
    }
}


void send_online_msg() {
    //get all the pot values and send the data
    String online_response = "";
    for(int i = 0; i < 5; i++) 
    {
        //online_response += armServos[i].potVal;
        online_response += armServos[i].currentPosition;
        //need to remove the extra ':' at the end
        if (i < 4) 
        {
            online_response += ":";
        }
    }
    Serial.print(online_response);
}


void parser(String s) {
    //check string has values
    if (s.length() >= 1) {

        if (s.startsWith("online", 1))
        {
            send_online_msg();
            return;
        }

        else
        {
            //need more validation here, check if the format is correct for sending joint target angles
            set_target_positions(s);
        }
    }
}


void move_servo(ServoMotor &servo) {
  int dir = 1;
  if (servo.currentPosition > servo.targetPosition) 
  {
    dir = -1;
  }

  int diff = abs(servo.currentPosition - servo.targetPosition);

  if (diff > 2) 
  {
     int new_position = servo.currentPosition + (5 * dir);
     int pulse = map(new_position, servo.minPosition, servo.maxPosition, servo.minpulse, servo.maxpulse); 
     pwm.setPWM(servo.pin, 0, pulse);  // increment/decrement the pulsewidth by the increment variable
     int measured_position = map(analogRead(servo.potPin), 
                                            servo.potMin, 
                                            servo.potMax, 
                                            servo.minPosition, 
                                            servo.maxPosition);
     servo.currentPosition = new_position;
    // Serial.print("Moving servo on pin ");
    // Serial.print(servo.pin);
    // Serial.print(" to ");
    // Serial.println(new_position);
     delay(delay_time);
  }

  else
  {
    return;
  }
}

//TODO
//add check for servo overloaded
//if the pot value hasn't changed when moving then servo is stuck
//after several attempts reset the servos position to the pot value
//send an warning message


void loop() {
  String response = "<";

  //set each of the servos current pot values
  for (int i = 0; i < 5; i++) 
  {
    int mapped_pot_val = map(analogRead(armServos[i].potPin), 
                                        armServos[i].potMin, 
                                        armServos[i].potMax, 
                                        armServos[i].minPosition, 
                                        armServos[i].maxPosition);
    armServos[i].potVal = mapped_pot_val;
    //Serial.print("joint:");
    //Serial.print(i);
    //Serial.print("  ");
    //Serial.print(mapped_pot_val);
    //Serial.print(" target: ");
    //Serial.println(armServos[i].targetPosition);
  }

  //Read in commands and parse the data
  if (Serial.available() > 0) 
  {
    String command_data = Serial.readString();
    command_data.trim();

    if(command_data.startsWith("<") && command_data.endsWith(">"))
    {
        parser(command_data);
    }

    Serial.flush();
  }

  //if a servo is not at it's target, increment it towards it's target
 for (int s = 0; s < 5; s++) 
 {

   if (armServos[s].targetPosition != armServos[s].currentPosition) 
   {
     move_servo(armServos[s]);
   }

   response += String(armServos[s].currentPosition);

   if (s == sizeof(armServos) / sizeof(armServos[0]) - 1) 
   {
     response += ">";
     break;
   } 

   else 
   {
     response += ":";
   }
 }

  if (response != prev_response)
  { 
      if (response.length() > 0 && response.begin() != ":") 
      {
          Serial.println(response);
          prev_response = response;
      }
  }

  Serial.flush();
  delay(25);
}
