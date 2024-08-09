#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ArduinoOTA.h>


#define SERVO_FREQ 50
const char* ssid = "";
const char* pwd = "";

WiFiServer server(5000);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int delay_time = 5;

struct RobotJoint 
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

//array of joints
RobotJoint armJoints[5] = {
  { 0, 420, 135, 180, 0, 0, 0, 180, 0, 4000, 800, 14 }, 
  { 1, 230, 490, 90, 0, 0, 0, 180, 0, 3350, 450, 27 },  
  { 2, 100, 490, 0, 0, 0,-90, 90, 0, 3250, 420, 26 },  
  { 3, 120, 490, 0, 0, 0,-90, 90, 0, 3200, 375, 25 },  
  { 4, 200, 400, 0, 0, 0, 0, 180, 0, 0, 0, 0 },         
};

//last response sent
String prev_response = "";

void setup() {

//start serial
//initialize the servo controller
  Serial.begin(115200);
  Serial.setTimeout(50);

  WiFi.begin(ssid,pwd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  server.begin();

  ArduinoOTA.setHostname("robot-arm");
  ArduinoOTA.setPassword("");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else if (ArduinoOTA.getCommand() == U_SPIFFS) {
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    switch (error) {
      case OTA_AUTH_ERROR:
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        Serial.println("End Failed");
        break;
    }
  });
  ArduinoOTA.begin();
  
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
    armJoints[i].targetPosition = armJoints[i].defaultPosition;
    int current_pos = analogRead(armJoints[i].potPin);

    current_pos = map(current_pos, 
                      armJoints[i].potMin, 
                      armJoints[i].potMax, 
                      armJoints[i].minPosition, 
                      armJoints[i].maxPosition);

    armJoints[i].currentPosition = current_pos;

    int pulse = map(current_pos, 
                    armJoints[i].minPosition, 
                    armJoints[i].maxPosition, 
                    armJoints[i].minpulse, 
                    armJoints[i].maxpulse);

    pwm.setPWM(armJoints[i].pin, 0, pulse);
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
            armJoints[number_of_joints].targetPosition = temp_str.toInt();
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
        //online_response += armJoints[i].potVal;
        online_response += armJoints[i].currentPosition;
        //need to remove the extra ':' at the end
        if (i < 4) 
        {
            online_response += ":";
        }
    }
    Serial.print(online_response);
}


void parser(String s) {
    if (s.startsWith("online", 1))
    {
        send_online_msg();
        return;
    }

    else
    {
        set_target_positions(s);
    }
}

bool check_valid_command(String command_string) {
    if(!command_string.startsWith("<") && !command_string.endsWith(">"))
    {
        return false;
    }
    else
    {
        return true;
    }
}


void move_servo(RobotJoint &servo) {
  int dir = 1;
  if (servo.currentPosition > servo.targetPosition) 
  {
    dir = -1;
  }

  int diff = abs(servo.currentPosition - servo.targetPosition);

  if (diff > 0) 
  {
     int new_position = servo.currentPosition + (1 * dir);
     int pulse = map(new_position, servo.minPosition, servo.maxPosition, servo.minpulse, servo.maxpulse); 
     pwm.setPWM(servo.pin, 0, pulse);  // increment/decrement the pulsewidth by the increment variable
     int measured_position = map(analogRead(servo.potPin), 
                                            servo.potMin, 
                                            servo.potMax, 
                                            servo.minPosition, 
                                            servo.maxPosition);
     servo.currentPosition = new_position;
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
    int mapped_pot_val = map(analogRead(armJoints[i].potPin), 
                                        armJoints[i].potMin, 
                                        armJoints[i].potMax, 
                                        armJoints[i].minPosition, 
                                        armJoints[i].maxPosition);
    armJoints[i].potVal = mapped_pot_val;
  }

  //Read in commands validate, parse the data
  if (Serial.available() > 0) 
  {
    String command_data = Serial.readString();
    command_data.trim();
    bool valid_command = check_valid_command(command_data);

    if (valid_command)
    {
        parser(command_data);
    }

    else
    {
        Serial.println("<Invalid command string>");
        Serial.flush();
    }
  }

  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    while (client.connected()) {
      if (client.available()) {
        // Read data from the client
        String wifi_data = client.readStringUntil('\n');
        bool valid_command = check_valid_command(wifi_data);
        if (valid_command)
        {
          parser(wifi_data);
        }
        else {
          Serial.println("<Invalid command string>");
          Serial.flush();
        }

        // Optionally send a response to the client
        //client.println("Data received");
      }
    }
    // Close the connection
    client.stop();
  }

//if a servo is not at it's target, increment it towards it's target
 for (int s = 0; s < 5; s++) 
 {

   if (armJoints[s].targetPosition != armJoints[s].currentPosition) 
   {
     move_servo(armJoints[s]);
   }

   response += String(armJoints[s].currentPosition);

// check if we are at the last joint, excluding gripper motor
// if last, append the end of message character
// else append the colon, expecting another value to append
   if (s == sizeof(armJoints) / sizeof(armJoints[0]) - 1) 
   {
     response += ">";
     break;
   } 

   else 
   {
     response += ":";
   }
 }

// check if the last response was the same as the current
  if (response != prev_response)
  { 
      if (response.length() > 0 && response.begin() != ":") 
      {
          Serial.println(response);
          prev_response = response;
      }
  }

  Serial.flush();
  delay(5);

  ArduinoOTA.handle();
}
