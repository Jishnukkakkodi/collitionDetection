
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>

#define steeringSensorPin A0
#define speedSensorPin 4

double steeringSensorValue;
static float steeringSensorOffset = 511.5;
double calibratedSteeringSensorValue;
double modifiedCalibratedSteeringSensorValue;
double findAngle;
static double angleConvertor = 0.293255132;

unsigned long prevmillis = 0;
unsigned long duration;
unsigned long refresh;
double rpm;
boolean currentstate;
boolean prevstate = LOW;

RF24 radio(9,10); // (CE,CSN)
const byte addresses[][6] = {"00001", "00002"};

SoftwareSerial bluetooth(2,3); // (Rx,Tx) 
String readString;

struct SensorData{
  double myAngle;
  double myrpm;
  String mylat;
  String mylon;
};

SensorData sensorData;

struct RcData{
  double rcAngle;
  double rcrpm;
  String rclat;
  String rclon;
};

RcData rcData;

void initialiseMyData(){
sensorData.myAngle = 0;
sensorData.myrpm = 0;
sensorData.mylat = "0";
sensorData.mylon ="0" ;
}

void initialisercData(){
  rcData.rcAngle = 0;
  rcData.rcrpm = 0;
  rcData.rclat = "0";
  rcData.rclon = "0";
}


void steeringWheelAngle(){
    steeringSensorValue = analogRead(steeringSensorPin);
    calibratedSteeringSensorValue = steeringSensorValue - steeringSensorOffset;
    if(calibratedSteeringSensorValue){
      findAngle = calibratedSteeringSensorValue * angleConvertor;
      sensorData.myAngle =findAngle;
    }else{ 
       sensorData.myAngle = 0;
    }
}

void vehicleSpeed(){
  currentstate = digitalRead(speedSensorPin);
  if(prevstate != currentstate){
    if(currentstate == HIGH){
      duration = ( micros() - prevmillis );
      rpm = (60000000/duration);
       prevmillis = micros();
    }
  }
  prevstate = currentstate;

  if( ( micros()- prevmillis ) > duration ){
    rpm = 0;
  }
  sensorData.myrpm = rpm;
}
void getGPSLocation(){
 while (bluetooth.available()) {
    delay(10);  //small delay to allow input buffer to fill

    char c = bluetooth.read();  //gets one byte from serial buffer
    if (c == ',') {
      break;
    }  //breaks out of capture loop to print readstring
    readString += c;
  } //makes the string readString 

  if (readString.length() >0) {
    sensorData.mylat = readString;
    readString=""; //clears variable for new input
  }
  while (bluetooth.available()) {
    delay(10);  //small delay to allow input buffer to fill

    char c = bluetooth.read();  //gets one byte from serial buffer
    if (c == ',') {
      break;
    }  //breaks out of capture loop to print readstring
    readString += c;
  } //makes the string readString 

  if (readString.length() >0) {
    sensorData.mylon = readString;
    readString=""; //clears variable for new input
  }
  
 }
 void SendData(){
  radio.stopListening();
  radio.write(&sensorData, sizeof(SensorData));
 }

 void RecieveData(){
  radio.startListening();
  while (!radio.available()){
    Serial.println("no nearby vehicle");
    Serial.print("safe");
    break;
  }
  radio.read(&rcData, sizeof(RcData));
 }

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(0, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
  
  pinMode(steeringSensorPin , INPUT);
  pinMode(speedSensorPin,INPUT);
  
  initialiseMyData();
  initialisercData();
}

void loop() {
  vehicleSpeed();
  getGPSLocation();
  steeringWheelAngle();
  SendData();
  RecieveData();
  
 
  Serial.println("===========My Data==========");
  Serial.println(sensorData.myrpm);
  Serial.println(sensorData.mylat);
  Serial.println(sensorData.mylon);
  Serial.println(sensorData.myAngle);
  Serial.println("===========Rc Data==========");
  Serial.println(rcData.rcrpm);
  Serial.println(rcData.rclat);
  Serial.println(rcData.rclon);
  Serial.println(rcData.rcAngle);
  delay(50);
}
