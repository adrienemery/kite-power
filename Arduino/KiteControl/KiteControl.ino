#include <Encoder.h>
#include <Servo.h>
#include <PID_v1.h>

#define TICKS_PER_REV 48

//*** Define Variables ***//

// Pan/Tilt 

Servo pan;
Servo tilt;
int newPanPos = 20;
int newTiltPos = 20;
int lastPanPos= 90;
int lastTiltPos = 90;


// Victor Motor Driver stuff

Servo leftVic;
Servo rightVic;
int maxSpeedVic = 127;
int minSpeedVic = 0;
int zeroPointVic = 127;
int leftVicPin = 9;
int rightVicPin = 8;

// Define Encoder Variables
Encoder knobLeft(20,21);
Encoder knobRight(18,19);

long currentLeft, currentRight;
long positionLeft  = 1;
long positionRight = 1;
long powerState = 96;
long newPowerState = 96;

long error1 = 0;
long error2 = 0;

// PID stuff defined here
double Setpoint, Input, Output;
double Kp,Ki,Kd = 1.0;

double leftSet, leftInput, leftOutput;
double lastLeftSet, lastLeftInput, lastLeftOutput;
double Kpl,Kil,Kdl = 1.0;

double rightSet, rightInput, rightOutput;
double lastRightSet, lastRightInput, lastRightOutput;
double Kpr,Kir,Kdr = 1.0;

PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);
PID leftPID(&leftInput, &leftOutput, &leftSet, Kpl, Kil, Kdl, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSet, Kpr, Kir, Kdr, DIRECT);

// Serial Contol Variables

const char notWorking = 'z';
const char changeKvals = 'k';
const char getInfo = 'i';
const char leftCalLeft = 'a';
const char leftCalRight = 'b';
const char rightCalLeft = 'c';
const char rightCalRight = 'd';
const char stopMotors = 'S';
const char autoPilotON = 'L';
const char autoPilotOFF = 'l';
const char zeroAzimuth = 'e';
const char rollSetpointCommand = 'R';  // used to send new setpoint
const char rollUpdate = 'r';           // used to update current roll angle
const char turn = 't';
const char power = 'p';
const char depower = 'd';
const char Speed = 's';
const char terminateChar = '/';
const char handshake = 'x';
const char panCommand = 'P';
const char tiltCommand = 'T';


String command = "";
String positionStr = "";
String motorSpeedStr = "";
String powerStr = "";

int lastState = 0;
int currentState =0;
char incomingByte = 'a';
int count = 0;

long leftMax = 20*TICKS_PER_REV;
long rightMax = 20*TICKS_PER_REV;

float powerMax = 500;
float depowerMax = 500;
float horizontalCount = 50;
float deltaX = 0.001;
long newPositionR = 96;
long newPositionL = 96;
long lastPosition = 0;
long turnLeftPosition = 0;
long turnRightPosition = 0;
long currentPosition = 0;
float howFar = 0.0;


// Motor pin connections for PWNM control 
//  ** THESE PINS MUST BE INITIALIZED ** // 

// TODO - ASSIGN PIN MAPPINGS TO MOTOR DRIVE PINSS

int pwmPinL = 10;
int pwmPinR = 11;

int leftMotorLeft = 28;
int leftMotorRight = 29;

int rightMotorLeft = 44;
int rightMotorRight = 45;

int maxSpeed = 255;
int motorSpeed = 255;

boolean calibrating = false;
boolean powerChange = false;
boolean leftMovingLeft = false;
boolean leftMovingRight = false;
boolean rightMovingLeft = false;
boolean rightMovingRight = false;
boolean movingLeft = false;
boolean movingRight = false;
boolean changeDirection = false;
boolean atSetPoint = false;
boolean print2screen = true;
boolean bprintData = false;
boolean bauto = false;
boolean usingVictorDrivers = false;

// Define Functions 

void leftMotorL(int motorSpeed = maxSpeed)
{
  if(usingVictorDrivers){
    motorSpeed = zeroPointVic - motorSpeed;
    leftVic.write(motorSpeed);
  }
  else{

    if(motorSpeed >255) motorSpeed = 255;
    if(motorSpeed < 0) motorSpeed = 0;
  }

  digitalWrite(leftMotorLeft,HIGH);
  digitalWrite(leftMotorRight, LOW);
  analogWrite(pwmPinL, motorSpeed);
}

void rightMotorL(int motorSpeed = maxSpeed)
{
  if(usingVictorDrivers){
    motorSpeed = zeroPointVic - motorSpeed;
    rightVic.write(motorSpeed);

  }
  else{

    if(motorSpeed >255) motorSpeed = 255;
    if(motorSpeed < 0) motorSpeed = 0;

    digitalWrite(rightMotorLeft, HIGH);
    digitalWrite(rightMotorRight, LOW);
    analogWrite(pwmPinR, motorSpeed);
  }

}


void leftMotorR(int motorSpeed = maxSpeed)
{

  if(usingVictorDrivers){
    motorSpeed = zeroPointVic - motorSpeed;
    leftVic.write(motorSpeed);

  }
  else{

    if(motorSpeed >255) motorSpeed = 255;
    if(motorSpeed < 0) motorSpeed = 0;

    digitalWrite(leftMotorLeft, LOW);
    digitalWrite(leftMotorRight,HIGH);
    analogWrite(pwmPinL, motorSpeed);
  }

  //Serial.print("Left turning Right: :");
  //Serial.println(motorSpeed);

}

void rightMotorR(int motorSpeed = maxSpeed)
{

  if(usingVictorDrivers){
    motorSpeed = zeroPointVic - motorSpeed;
    rightVic.write(motorSpeed);

  }
  else{

    if(motorSpeed >255) motorSpeed = 255;
    if(motorSpeed < 0) motorSpeed = 0;

    digitalWrite(rightMotorLeft, LOW);
    digitalWrite(rightMotorRight, HIGH);
    analogWrite(pwmPinR, motorSpeed);
    //Serial.print("Rigth turning Right: :");
    //Serial.println(motorSpeed);
  }
}


void holdPosition()
{
  // TODO 
  digitalWrite(leftMotorLeft, LOW);
  digitalWrite(leftMotorRight,LOW);
  analogWrite(pwmPinL, 0);

  digitalWrite(rightMotorLeft, LOW);
  digitalWrite(rightMotorRight, LOW);
  analogWrite(pwmPinR, 0);

}

void stopLeft()
{
  if(usingVictorDrivers){

    leftVic.write(zeroPointVic);  // stops leftMotor

  }
  else{

    digitalWrite(leftMotorLeft, LOW);
    digitalWrite(leftMotorRight,LOW);
    analogWrite(pwmPinL, 0);
  }
}

void stopRight()
{
  if(usingVictorDrivers){
    rightVic.write(zeroPointVic);

  }
  else{

    digitalWrite(rightMotorLeft, LOW);
    digitalWrite(rightMotorRight, LOW);
    analogWrite(pwmPinR, 0);
  }
}

void printData(){
  Serial.print("\nInput: ");
  Serial.print(Input);
  Serial.print(" Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(" Output:  ");
  Serial.println(Output);

  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print("  Ki: ");
  Serial.print(Ki);
  Serial.print("  Kd: ");
  Serial.println(Kd); 

  Serial.print("L: ");
  Serial.print(turnLeftPosition);
  Serial.print("  R: ");
  Serial.println(turnRightPosition);

  Serial.print("Left Encoder: ");
  Serial.print(currentLeft);
  Serial.print(" Right Encoder: ");
  Serial.println(currentRight);

  bprintData = false;

}


void setup() {

  Serial.begin(115200);   // open serial port 
  Serial.flush();
  Serial.println("Kite Control 1.23");

  // Initialize PIDs
  Setpoint = 0;
  Input = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-30,30);
  myPID.SetSampleTime(100);
  myPID.SetTunings(1,0,0);

  leftSet = 96;
  leftInput = 96;
  Kpl = 4.0;
  Kdl = 0.0;
  Kil = 0.0;

  rightSet = 96;
  rightInput = 96;
  Kpr = 4.0;
  Kir = 0.0;
  Kdr = 0.0;

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-127,127);
  leftPID.SetSampleTime(200);
  leftPID.SetTunings(Kpl,Kdl,Kil);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-127,127);
  rightPID.SetSampleTime(200);
  rightPID.SetTunings(Kpr,Kir,Kdr);

  knobLeft.write(TICKS_PER_REV*12);
  knobRight.write(TICKS_PER_REV*12);

  // initialize victor motor driver pins
  leftVic.attach(leftVicPin);
  rightVic.attach(rightVicPin);

  // initialize pan/tilt servos
  pan.attach(7);
  tilt.attach(5);

  pan.write(90);
  tilt.write(90);
  
  // Setup pin modes
  pinMode(leftMotorLeft, OUTPUT);
  pinMode(leftMotorRight, OUTPUT);
  pinMode(rightMotorRight, OUTPUT);
  pinMode(rightMotorLeft, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(pwmPinL, OUTPUT);
  
  // set motor speeds to zero
  stopLeft();
  stopRight();

}

void loop() {

  count ++;
  if(count == 150){
    count = 0;
  }

  // Do stuff if data is available to read
  if(Serial.available() > 0)
  {
    command = "";
    incomingByte = Serial.read();

    while(incomingByte != terminateChar)  // Read from serial until terminating char is recieved
    {
      if(Serial.available() > 0 ){
        command += incomingByte;    // concatonate command with incoming byte
        incomingByte = Serial.read();  // Read byte from serial port
      }

      //      if(abs(t0 - millis()) > 2000){  // break out of serial reading after 2 seconds 
      //        command = " ";
      //        break;
      //      }

    }
    Serial.print('r');        // done reading data, so give PC ok to write more data   

    // Check first index in incoming command to determine the command given
    if(command[0] == turn)
    {  
      positionStr = command.substring(1);  // assigns rest of position command string 
      turnLeftPosition = - long(positionStr.toFloat());  // convert to float   
      turnRightPosition = -turnLeftPosition;
      // check that turn command doesn't exceed max turn
      //      if(turnLeftPosition > 20) turnLeftPosition = 20;
      //      if(turnLeftPosition < -20) turnLeftPosition = -20;
      //      if(turnRightPosition > 20) turnRightPosition = 20;
      //      if(turnRightPosition < -20) turnRightPosition = -20;

      newPositionL = turnLeftPosition + powerState;
      newPositionR = turnRightPosition + powerState;

    } 
    else if(command[0] == power){
      powerStr = command.substring(1);
      newPowerState = int(powerStr.toFloat());
      powerState = newPowerState;
      powerChange = true;
      newPositionL = turnLeftPosition + newPowerState;
      newPositionR = turnRightPosition + newPowerState;

    } 
    else if(command[0] == Speed) {
      motorSpeedStr = command.substring(1);  // assings rest of speed command to string
      motorSpeed = int(motorSpeedStr.toFloat()); // convert to int
      if(motorSpeed > 255) motorSpeed = 255;
      if(motorSpeed < 0 ) motorSpeed = 0;
      // TODO do stuff with user input for moto speed
      //Serial.print("New motor speed: ");
      //Serial.println(motorSpeed);

    } 
    else if(command[0] == leftCalLeft) {
      leftMotorL(75);
      calibrating = true;

    } 
    else if(command[0] == leftCalRight){  
      leftMotorR(75);
      calibrating = true;

    } 
    else if(command[0] == rightCalLeft){           
      rightMotorL(75);           
      calibrating = true;

    } 
    else if(command[0] == rightCalRight){
      rightMotorR(75); 
      calibrating = true;

    } 
    else if(command[0] == stopMotors){    // reset all values to default 
      stopLeft();
      stopRight(); 
      delay(1000);
      knobLeft.write(TICKS_PER_REV*12);
      knobRight.write(TICKS_PER_REV*12);
      powerState = 96;
      newPositionL = 96;
      newPositionR = 96;
      calibrating = false;

    } 
    else if(command[0] == autoPilotON){
      //rollPID.SetMode(AUTOMATIC);  // turn ON roll PID
      bauto = true;
      myPID.SetMode(AUTOMATIC);
      //Serial.println("AUTOPILOT ON");

    } 
    else if(command[0] == autoPilotOFF){
      //rollPID.SetMode(MANUAL);  // turn OFF roll PID
      bauto = false;
      myPID.SetMode(MANUAL);
      //Serial.println("AUTOPILOT OFF");


    } 
    else if(command[0] == rollSetpointCommand && bauto){
      Setpoint = double(command.substring(1).toFloat());   // update setpoint
      //Serial.print("roll setpoint: ");
      Serial.println(Setpoint);


    } 
    else if(command[0] == rollUpdate && bauto){
      Input = double(command.substring(1).toFloat());  // update current roll angle
      //Serial.print("roll input: ");
      Serial.println(Input);

    }
    else if(command[0] == getInfo){   
      bprintData = true; 

    }
    else{
      Serial.println("Invalid command!");

    }
  }


  // Compute PID output to minimze error in roll angle when in autopilot mode 
  //  if(bauto){
  //    myPID.Compute();
  //    turnLeftPosition = - long(Output);  
  //    turnRightPosition = -turnLeftPosition;
  //    newPositionL = turnLeftPosition + powerState;
  //    newPositionR = turnRightPosition + powerState;
  //
  //  }

  /* 
   ENCODER TRACKING ALGORITHM
   1. Determine Current position 
   2. Compare current Position to setPostion
   3. Use PID to calculat motor speed required to get to new position 
   4. If current setPosition is less than current position Move left
   5. Else Move right
   
   */

  // read in current encoder position
  currentLeft = knobLeft.read();    
  currentRight = knobRight.read();

  // Convert to 1/8th revolution resolution 
  double numRevs = currentLeft/TICKS_PER_REV;
  double numHalfRevs = currentLeft/(TICKS_PER_REV/8);
  double numHalfRevsRight = currentRight/(TICKS_PER_REV/8);

  // Update PID input values
  //  leftInput = numHalfRevs;
  //  rightInput = numHalfRevsRight;
  //
  //  // Compute new PID outputs
  //  leftPID.Compute();
  //  rightPID.Compute();
  //  myPID.Compute();
  //
  //  // print output values if they have changed
  //  if(lastLeftOutput != leftOutput || lastRightOutput != rightOutput){
  //    //     Serial.print("Left PID output: ");
  //    //     Serial.print(leftOutput);
  //    //     Serial.print("  Right PID output: ");
  //    //     Serial.println(rightOutput);
  //    //     Serial.print("Revs - L:");
  //    //     Serial.print(numHalfRevs);
  //    //     Serial.print(" R: ");
  //    //     Serial.println(numHalfRevsRight);
  //    //     Serial.print("Left setpoint: ");
  //    //     Serial.print(leftSet);
  //    //     Serial.print("  Right setpoint: ");
  //    //     Serial.println(rightSet);
  //  }
  //
  //  // update last output readings
  //  lastLeftOutput = leftOutput;
  //  lastRightOutput = rightOutput;
  //
  //
  //  leftOutput = -leftOutput;
  //  rightOutput = - rightOutput;


  if(!calibrating)
  {     
    if(usingVictorDrivers){

      if(newPositionL < numHalfRevs){    // reel IN line 
        leftMotorL(leftOutput);         
      } 
      else if(newPositionL > numHalfRevs){  // reel OUT line
        leftMotorR(leftOutput); 
      }
      if(newPositionR < numHalfRevsRight){  // reel OUT line
        rightMotorR(rightOutput);    
      } 
      else if(newPositionR > numHalfRevsRight){  // reel IN line
        rightMotorL(rightOutput);       
      }

      if(newPositionL == numHalfRevs){        // stop left motor
        stopLeft();  
      } 
      if(newPositionR == numHalfRevsRight) { // stop right motor
        stopRight();  
      }

      if(abs(newPositionL - numHalfRevs) <= 2) {  // lowers motor speed if close to setpoint
        stopLeft();
      }

      if(abs(newPositionR - numHalfRevsRight) <= 2) {  // lowers motor speed if close to setpoint
        stopRight();
      }

    }
    else{

      if(newPositionL < numHalfRevs){    // reel IN line 
        leftMotorL(motorSpeed);  
        //Serial.println("Left Turning Left");      
      } 
      else if(newPositionL > numHalfRevs){  // reel OUT line
        leftMotorR(motorSpeed); 
        //Serial.println("Left Turning Right");  
      }
      
      if(newPositionR < numHalfRevsRight){  // reel OUT line
        rightMotorR(motorSpeed); 
        //Serial.println("Right Turning Right");     
      } 
      else if(newPositionR > numHalfRevsRight){  // reel IN line
        rightMotorL(motorSpeed);  
        //Serial.println("Right Turning Left");     

      }

      if(newPositionL == numHalfRevs){
        stopLeft();  

      } 
      if(newPositionR == numHalfRevsRight) {
        stopRight();  
      }

      if(abs(newPositionL - numHalfRevs) <= 1) {  // lowers motor speed if close to setpoint
        //stopLeft();
        motorSpeed = 100;
      }
      else {
        motorSpeed = 255; 
      }

      if(abs(newPositionR - numHalfRevsRight) <= 1) {  // lowers motor speed if close to setpoint
        //stopRight();
        motorSpeed = 100;
      }
      else {
        motorSpeed = 255; 
      }
    }

    if(numHalfRevs!= positionLeft || numHalfRevsRight != positionRight)
    {
      Serial.print("Revs - L:");
      Serial.print(numHalfRevs);
      Serial.print(" R: ");
      Serial.println(numHalfRevsRight);
      Serial.print("Pos - L:");
      Serial.print(newPositionL);
      Serial.print(" R:");
      Serial.println(newPositionR);
      
      
      positionLeft = numHalfRevs; 
      positionRight = numHalfRevsRight;
    }  
  }     

  delay(20);

  if(bprintData){
    printData();
  }

  // TODO 
  // Check battery levels of each cell in LiPo Pack
  // if they are low cut power to motors and send Message to PC 


}



