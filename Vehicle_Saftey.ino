#define BLYNK_PRINT Serial

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

TinyGPSPlus gps;
WidgetMap myMap(V0);

SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial EspSerial(2, 3);

BlynkTimer timer;

float spd;
float latitude;
float longitude;
bool alcFlag = true;
bool accFlag = true;

#define alcohol A0
#define led 13
#define buzzer 12
#define relayPin 11
#define vibration 8

#define samples 10
int xSample = 0;
int ySample = 0;
int zSample = 0;

#define minVal -10
#define MaxVal 10
#define vibrationThres 1000
#define sensorThres 400

#define ESP8266_BAUD 9600
ESP8266 wifi(&EspSerial);

char auth[] = "NAvEEgviR3ZL9gcpR_VLoR_pkzZFKiba";
char ssid[] = "DRT";
char pass[] = "Avengers321";

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  EspSerial.begin(ESP8266_BAUD);

  delay(100);

  pinMode(alcohol, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(relayPin, OUTPUT);

  Blynk.begin(auth, wifi, ssid, pass);
  if (!accel.begin())
  {
    Serial.println("No valid sensor found");
    while (1);
  }

  sensors_event_t event;
  accel.getEvent(&event);
  for (int i = 0; i < samples; i++)
  {
    xSample += event.acceleration.x;
    ySample += event.acceleration.y;
    zSample += event.acceleration.z;
  }
  xSample /= samples;
  ySample /= samples;
  zSample /= samples;

  digitalWrite(relayPin, LOW);
  digitalWrite(led, HIGH);

  timer.setInterval(1000L, checkData);
}

void checkData() {
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }

  int alcoholData = analogRead(alcohol);
  Blynk.virtualWrite(V1, alcoholData);

  if ((alcoholData >= sensorThres) && (alcFlag == true)) {
    Blynk.notify("Alcohol Detected");
    tone(buzzer, 1000, 1000);
    digitalWrite(led, LOW);
    alcFlag = false;
    if (spd > 0) {
      delay(3000);
      digitalWrite(relayPin, HIGH);
    }
    else {
      digitalWrite(relayPin, HIGH);
    }
  }
  if (alcoholData < sensorThres) {
    digitalWrite(led, HIGH);
    noTone(buzzer);
    digitalWrite(relayPin, LOW);
    alcFlag = true;
  }
  sensors_event_t event;
  accel.getEvent(&event);

  int xValue = xSample - event.acceleration.x;
  int yValue = ySample - event.acceleration.y;
  int zValue = zSample - event.acceleration.z;

  int vibrationValue = digitalRead(vibration);

  if (((xValue < minVal || xValue > MaxVal  || yValue < minVal || yValue > MaxVal  || zValue < minVal || zValue > MaxVal || vibrationValue > vibrationThres)) && (accFlag == true)) {
    Blynk.notify("Accident Alert!!! Please respont to the current vehicle location...");
    String accMessage = "Your vehicle experienced accident at following location: https://www.google.com/maps/search/?api=1&query=" + String(latitude) + "," + String(longitude) + " Please respond help immediately.";
    Blynk.email("drt347826@gmail.com", "Accident Alert", accMessage);
    digitalWrite(relayPin, HIGH);
    accFlag = false;
  }
}

void loop()
{
  while (ss.available() > 0)
  {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(ss.read()))
      displayInfo();
  }
  Blynk.run();
  timer.run();
}

void displayInfo()
{
  if (gps.location.isValid() )
  {
    latitude = (gps.location.lat(), 6);
    longitude = (gps.location.lng(), 6);
    myMap.location(1, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();
    Blynk.virtualWrite(V4, spd);
    delay(500);
  }
}
