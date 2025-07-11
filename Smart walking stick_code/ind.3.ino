#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
const int pulsePin = A0;
const int highThreshold = 750;
const int lowThreshold = 350;
int pulseValue = 0;
const long motionThreshold = 25000; 
bool buzzerOn = false;
#define ECHO1 2
#define TRIG1 3
#define ECHO2 4
#define TRIG2 5
#define ECHO3 6
#define TRIG3 7
#define BUZZER 8
#define pushbutton 9
int ldrPin = 10;
int ledPin2 = 11;
int ledPin3 = 12;
#define OBSTACLE_THRESHOLD 25
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;
SoftwareSerial ss(A2,A3);// RX, TX
float readDistance(int trigPin, int echoPin);   
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  ss.begin(9600);
  Serial.println("Waiting for GPS signal...");
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(pulsePin, INPUT);
  pinMode(9, INPUT_PULLUP);
}
void loop() {
  float dist1 = readDistance(TRIG1, ECHO1);
  float dist2 = readDistance(TRIG2, ECHO2);
  float dist3 = readDistance(TRIG3, ECHO3);
   int lightState = digitalRead(ldrPin);
     while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      int hour = gps.time.hour();
      int minute = gps.time.minute();
      int second = gps.time.second();
      hour += 5;
      minute += 30;
      if (minute >= 60) {
        minute -= 60;
        hour += 1;
      }
      if (hour >= 24) {
        hour -= 24;
      }
      Serial.println("Coordinates:");
      Serial.print(lat, 6);
      Serial.print(", ");
      Serial.println(lng, 6);
      Serial.print("Google Maps: https://www.google.com/maps?q=");
      Serial.print(lat, 6);
      Serial.print(",");
      Serial.println(lng, 6);
      Serial.print("Time (IST): ");
      if (hour < 10) Serial.print("0");
      Serial.print(hour);
      Serial.print(":");
      if (minute < 10) Serial.print("0");
      Serial.print(minute);
      Serial.print(":");
      if (second < 10) Serial.print("0");
      Serial.println(second);
      Serial.println("------------------------");
    }
  }
  if (lightState == LOW) {
    digitalWrite(ledPin2, LOW );  
    digitalWrite(ledPin3, LOW ); 
  } else {
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
  }
  pulseValue = analogRead(pulsePin);
  Serial.println(pulseValue);
  if (pulseValue > highThreshold || pulseValue < lowThreshold || (dist1 > 0 && dist1 <= OBSTACLE_THRESHOLD) ||
      (dist2 > 0 && dist2 <= OBSTACLE_THRESHOLD) ||
      (dist3 > 0 && dist3 <= OBSTACLE_THRESHOLD)) {
    digitalWrite(BUZZER, HIGH);
    delay(200);
      }
      else{
    digitalWrite(BUZZER, LOW);
  }
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  long totalAccel = abs(ax) + abs(ay) + abs(az);
  Serial.println("Accel sum: " + String(totalAccel));
  if(totalAccel > motionThreshold){
    digitalWrite(BUZZER, HIGH);
    delay(5000);
  }
   if(digitalRead(pushbutton) == LOW){
  digitalWrite(BUZZER, HIGH);
  delay(5000);
    digitalWrite(BUZZER, LOW);
 }
 }

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0;
  return distance;
}