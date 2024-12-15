/*#include <SoftwareSerial.h>
SoftwareSerial Serial(0,1); // RX, TX*/

//driving mode of the car, it depends on tha band of the of the track
# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2
# define BIG_ANGLE_LEFT 3
# define BIG_ANGLE_RIGHT 4
# define SMALL_ANGLE_LEFT 5
# define SMALL_ANGLE_RIGHT 6

//motor pin configuration
const int rightMotorEnable = 10;
const int leftMotorEnable = 11;
const int rightMotorBackward = A5;
const int rightMotorForward = A4;
const int leftMotorBackward = 12;
const int leftMotorForward = 13;

const int lineFollowSensor0 = 6; //righ most sensor
const int lineFollowSensor1 = 5; 
const int lineFollowSensor2 = 4; 
const int lineFollowSensor3 = 3;
const int lineFollowSensor4 = 7; //left most sensor

const int Interrupt= 2;
const int Relay= A3;
const int busPin= A2;
const int outPin= A1;
const int signPin=A0;

const int Echo= 8;
const int Trigger= 9;

int leftMotorStartingSpeed = 70;
int rightMotorStartingSpeed = 80;
int manualSpeed = 65;


int mode = 0;
unsigned char IntCount=0;
unsigned char VolCount=0;
int distance;
bool obstacle;


int LFSensor[5]={0, 0, 0, 0, 0};

// PID controller
float Kp=25;
float Ki=0;
float Kd=15;

float pidValue = 0;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

int wheelCheck=0;
//String command;
String device;

String inputString = "";
String command = "";
String value = "";
boolean stringComplete = false; 

void setup() {
  Serial.begin(9650);  //Set the baud rate to that of your Serial module.
  //Serial.begin(9650); //set the baud rate for your Serial module

  //all motor controller related pin should be output
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorEnable, OUTPUT);
  pinMode(leftMotorEnable, OUTPUT);
  pinMode(Relay,OUTPUT);
  pinMode(Trigger, OUTPUT);


  //all sensor pin should be input
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);
  pinMode(Interrupt,INPUT_PULLUP);
  pinMode(Echo, INPUT);
  pinMode(signPin, INPUT);
  pinMode(outPin, INPUT);
  pinMode(busPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), Start, FALLING); // Interrupt


  //command for Serial receiver
  inputString.reserve(50);  // reserve 50 bytes in memory to save for string manipulation 
  command.reserve(50);
  value.reserve(50);
}

void loop() {

 //uncomment the following two lines for testing sensors
 //testLineFollowSensorsAndPIDvalue();
 //delay(1000);

 /****************This snippet is for PID tuning using Serial***************************/
 serialEvent(); //check if Serial receiver receives any data
 if (stringComplete) {
    delay(100);
    // identified the posiion of '=' in string and set its index to pos variable
    int pos = inputString.indexOf('=');
    // value of pos variable > or = 0 means '=' present in received string.
    if (pos > -1) {
      // substring(start, stop) function cut a specific portion of string from start to stop
      // here command will be the portion of received string till '='
      // let received string is KP=123.25
      // then the receive value id for KP and actual value is 123.25 
      command = inputString.substring(0, pos);
      // value will be from after = to newline command
      // for the above example Kp value is 123.25
      // we just ignoreing the '=' taking first parameter of substring as 'pos+1'
      // we are using '=' as a separator between command and vale
      // without '=' any other character can be used
      value = inputString.substring(pos+1, inputString.length()-1);  // extract command up to \n exluded
      pidValue = value.toFloat(); //convert the string into float value
      //uncomment following line to check the value in serial monitor
      //Serial.println(pidValue);
      if(command == "KP"){
           Kp = pidValue;
           delay(50);
           }
      else if(command == "KI"){
           Ki = pidValue;
           delay(50);
           }
      else if(command == "KD"){
           Kd = pidValue;
           delay(50);
           } 
        } 
      inputString = "";
      stringComplete = false;//all the data is collected from serial buffer
    }

//Every 20th loop Voltage Current Value will be displayed
VolCount++;
 if (VolCount==20){
   VoltageCurrent();
   VolCount=0;
 }
 //if Obstacle detected in path
 obstacle= Ultrasonic();
 if (obstacle){
  Serial.println("OBSTACLE Detected");
  Serial.println("180 Turn");
  do{
    int x=0;
    wheelCheck=0;
    wheel_rotation(65,-65); //rotate right for small angle
    if (x==0)
    delay(300);  //delay determines the degree of angle
    x++;        
  }while(digitalRead(lineFollowSensor2)==1);      
 }
 /*********************This snippet is for Line Following***************************************/
 readLFSsensorsAndCalculateError(); //read sensor, calculate error and set driving mode
 switch (mode)
   {
    case STOPPED:  //all sensors read black line
      wheelCheck=0;
      motorStop();
      previousError = error;
      Serial.println("STOPPED");
      break;

    case NO_LINE:  //all sensor are on white plane
      wheelCheck+=1; 
      if(wheelCheck==1){
      forward();
      delay(2);
      }
      else if(wheelCheck==2){
      wheel_rotation(65,-65);
      delay(4);
      }
      previousError = 0;
      Serial.println("NO LINE");
      break;
      
    case BIG_ANGLE_RIGHT:
      wheelCheck=0;
      wheel_rotation(65,-65); //rotate right for small angle
      delay(5);  //delay determines the degree of angle
      Serial.println("BIG RIGHT");
      break;

    case BIG_ANGLE_LEFT:
      wheelCheck=0;
      wheel_rotation(-75,75); //rotate left for small angle
      delay(5);
      Serial.println("BIG LEFT");
      break;
      
    case SMALL_ANGLE_RIGHT:
      wheelCheck=0;
      wheel_rotation(65,-65);
      delay(20); //rotate right for big angle
      previousError = 0;
      //Serial.println("SMALL RIGHT");
      break;

    case SMALL_ANGLE_LEFT:
      wheelCheck=0;
      wheel_rotation(-75,75);
      delay(20); //rotate left for big angle
      previousError = 0;
      //Serial.println("SMALL LEFT");
      break;

    case FOLLOWING_LINE: //everything is going well
      wheelCheck=0;      
      calculatePID();
      motorPIDcontrol();
      Serial.println("STRAIGHT");
      break;     
  }

}

void forward(){ //drive forward at a preset speed
  analogWrite(rightMotorEnable, manualSpeed);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  
  analogWrite(leftMotorEnable, manualSpeed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
}

void left(){ //left motor rotate CCW and right motor rotate CW
  analogWrite(rightMotorEnable, manualSpeed);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  
  analogWrite(leftMotorEnable, manualSpeed);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
}

void right(){ //left motor rotate CW and left motor rotate CCW
  analogWrite(rightMotorEnable, manualSpeed);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
  
  analogWrite(leftMotorEnable, manualSpeed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
}

void motorStop(){ //speed of both motor is zero
  analogWrite(rightMotorEnable, 0);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  
  analogWrite(leftMotorEnable, 0);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
}

void readLFSsensorsAndCalculateError()
{
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);
  LFSensor[3] = digitalRead(lineFollowSensor3);
  LFSensor[4] = digitalRead(lineFollowSensor4);
  
  if((     LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -4;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = FOLLOWING_LINE; error = -3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = -2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = -1;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 0;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 1;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = FOLLOWING_LINE; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = STOPPED; error = 0;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = NO_LINE; error = 0;}
  //track goes right at an angle >90 degree
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = BIG_ANGLE_RIGHT; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = BIG_ANGLE_RIGHT; error = 0;}
  //track goes right at an angle <90 degree
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = SMALL_ANGLE_RIGHT; error = 0;}
  //track goes left at an angle >90 degree
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = BIG_ANGLE_LEFT; error = 0;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = BIG_ANGLE_LEFT; error = 0;}
  //track goes left at an angle <90 degree
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = SMALL_ANGLE_LEFT; error = 0;}

}

void testLineFollowSensorsAndPIDvalue()
{
     int LFS0 = digitalRead(lineFollowSensor0);
     int LFS1 = digitalRead(lineFollowSensor1);
     int LFS2 = digitalRead(lineFollowSensor2);
     int LFS3 = digitalRead(lineFollowSensor3);
     int LFS4 = digitalRead(lineFollowSensor4);
     
     Serial.print ("LFS: L  0 1 2 3 4  R ==> "); 
     Serial.print (LFS0); 
     Serial.print (" ");
     Serial.print (LFS1); 
     Serial.print (" ");
     Serial.print (LFS2); 
     Serial.print (" ");
     Serial.print (LFS3); 
     Serial.print (" ");
     Serial.print (LFS4); 
     Serial.print ("  ==> ");
    
     Serial.print (" P: ");
     Serial.print (P);
     Serial.print (" I: ");
     Serial.print (I);
     Serial.print (" D: ");
     Serial.print (D);
     Serial.print (" PID: ");
     Serial.println (PIDvalue);
}

void calculatePID()
{
  P = error; //maximum error value is 4 and minimum is zero
  I = I + error; 
  D = error-previousError; 
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}

void motorPIDcontrol()
{  
  int leftMotorSpeed = leftMotorStartingSpeed + PIDvalue;
  int rightMotorSpeed = rightMotorStartingSpeed - PIDvalue;
  
  // The motor speed should not exceed the max PWM value
  constrain(leftMotorSpeed, 60, 100);
  constrain(rightMotorSpeed, 60, 100);

  //determing the rotation and direction of wheel
  wheel_rotation(leftMotorSpeed,rightMotorSpeed);
}

void wheel_rotation(int left, int right){
  if(left>0){
    analogWrite(leftMotorEnable, left);
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(leftMotorBackward, LOW);
    }
  else if(left<0){
    analogWrite(leftMotorEnable, abs(left));
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBackward, HIGH);
    }
  if(right>0){
    analogWrite(rightMotorEnable, right);
    digitalWrite(rightMotorForward, HIGH);
    digitalWrite(rightMotorBackward, LOW);
    }
  else if(right<0){
    analogWrite(rightMotorEnable, abs(right));
    digitalWrite(rightMotorForward, LOW);
    digitalWrite(rightMotorBackward, HIGH);
    }
  }

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    //Serial.write(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline or a carriage return, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } 
  }
}

void Start(){
  IntCount++;
  if(IntCount==1)
  {    
    digitalWrite(Relay, HIGH);
  }
  else{
    digitalWrite(Relay, LOW);
    IntCount=0;
  }
}

bool Ultrasonic(){
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(Echo, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back) 
  if ((distance>1) && (distance<=15))
  return 1;
  else
  return 0; 
}

void VoltageCurrent(){
  float voltage = 0.0;
  unsigned long rawVal = 0;
  analogReference(INTERNAL);
  /* After switching you have to execute a dummy analogRead, 
     followed by a short */
  rawVal = analogRead(outPin);
  rawVal = 0;
  delay(10);
  rawVal += analogRead(outPin);
  Serial.println("**********Battery Voltage Current************");
  voltage = rawVal * 1100.0 / 1023.0;
  Serial.print("V Out         [mV]: ");
  Serial.println(voltage);
  float current_mA = voltage * 0.9452 + 1.0544; // Calibration parameters
  Serial.print("Busstrom      [mA]: ");  // Bus current
  Serial.println(current_mA);
  Serial.print("Stromrichtung:    : ");  // Current direction
  if(digitalRead(signPin)){
    Serial.println("RS+ -> RS-");
  }
  else{
    Serial.println("RS- -> RS+");
  }
  analogReference(DEFAULT);
  voltage = analogRead(busPin) * 5.0 / 1023.0;  
  Serial.print("Busspannung    [V]: "); // Bus voltage
  Serial.println(voltage);
  float power_mW = (voltage * current_mA);
  Serial.print("Leistung      [mW]: ");  // Power
  Serial.println(power_mW);
  Serial.println("-------------------------------");
}