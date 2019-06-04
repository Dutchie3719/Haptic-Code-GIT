/*
Adafruit Arduino - Lesson 14. Knob

* This program captures accelerometerdata and provides initial calibration.

  Details
  =======

  Boards
  =====
  SparkFun Redboard

  Connections
  ===========
  Connect A0 to X Axis of Joystick
  Connect A1 to Y Axis of Joystick
  Connect VDD to 5V DC
  Connect GROUND to common ground
  Connect VIBRO to Pin 3 (PWM)

  History
  =======
  2015/MAR/03  - First release (SHOR)

  Instructions
  ============

  

*/



/*
Pin Declarations
=======
*/

int xpotPin = 0;
int ypotPin = 1;
int vibroPin = 3;
int readingx = 0;
int readingy = 0;

double duration;
 
void setup(){
   Serial.begin(115200);
   pinMode(ypotPin,INPUT);
   pinMode(xpotPin,INPUT);
   pinMode(vibroPin,OUTPUT);
}

void datacall(){
  readingx = analogRead(xpotPin);     // 0 to 1023
  readingy = analogRead(ypotPin);     // 0 to 1023

//  Serial.print("X Pot:");
//  Serial.println(readingx);
//
//  Serial.print("Y Pot:");
//  Serial.println(readingy);

  delay(30);
}

void datamath(){

  int adjx = abs(readingx - 512);
  int adjy = abs(readingy - 512);

  Serial.print("X Pot Adj:");
  Serial.println(adjx);

  Serial.print("Y Pot Adj:");
  Serial.println(adjy);
  

  double a = 0;
  double b = 0;
  double c = 0;

  adjx = adjx/10;
  adjy = adjy/10;
  
  a = adjx * adjx;
  b = adjy * adjy;
  c = sqrt(a+b);
  //Serial.println(c);

  if (c >= 51){
    c = 51;
  }

  duration = c*(5);
  
  Serial.print("PWM Value:");
  Serial.println(duration);

//  
//  Serial.print("X Pot Adj:");
//  Serial.println(adjx);
//
//  Serial.print("Y Pot Adj:");
//  Serial.println(adjy);
  
//  float theta = tan(adjy/adjx);
//  float thetadeg = (theta * 180)/ 3.14;
//  Serial.print("Theta: ");
//  Serial.print(thetadeg);
//
//  float polarx = adjx*cos(theta);
//  float polary = adjy*sin(theta);
//
//  float poldist = polarx+polary;
//
//  poldist = abs(poldist);
//  Serial.print(" | Radius: ");
//  Serial.println(poldist);
// 
//  duration = poldist * (255/512);  
  
}
void loop() 
{ 
  datacall();
  datamath();
  pwmplay();
  delay(2);
} 

void pwmplay()
{
  int pwmval = round(duration);
  analogWrite(3,pwmval);

}
