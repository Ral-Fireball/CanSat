#include <Adafruit_BMP280.h>  // voor de barometer
#include <Adafruit_MPU6050.h> //voor MPU6050
#include <Adafruit_Sensor.h> //voor MPU6050
#include <Wire.h>  // voor basicly alles, komt uit barometer
#include <SPI.h>  // barometer
#include <SoftwareSerial.h>  // komt uit gps
#include <TinyGPS++.h>  // voor gps
#include <Servo.h> //voor servo

Adafruit_BMP280 bmp; // I2C
Adafruit_MPU6050 mpu; //I2C

Servo release;

/*gebruikte pins:
1 - hardware serial (+ radio) rx radio
2 - hardware serial (+ radio) tx radio
3 - tx gps
4 - rx gps
5 -
6 - 
7 - LED PM sensor
8 - rx radio
9 -  tx radio
10 - servo
11 - momenteel satus led
12 - 
13 - ingebouwde arduino led
A0 - CO
A1 - NO2
A2 - NH3
A3 - O3 measure
A4 - SDA
A5 - !!!SCL - 
A6 - measure PM
A7
*/

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

const float max_volts = 5.0;
const float max_analog_steps = 1023.0;  // constantes voor analoge inputs

float temp;
float press;
float alt;  // variablen voor gemeten waarde barometer

float xacc; //variabelen voor de IMU
float yacc;
float zacc;
float totalAcceleration;

double lat;
double lng;
int hour;
int min;
int sec;
bool GPSUpdate;// variabelen voor gemeten waarde GPS

float CO;
float NH3;
float NO2;  // variabelen voor de gas sensor

int measurePin = A6; //de zwarte kabel moet in A5
int ledPower = 7; //witte kabel moet in gpio ... (in dit geval 7)
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;  // variabelen voor de pm sensor

int O3_pin = A3;
float O3;

float last_alt;
float new_alt;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

SoftwareSerial radio(8, 9);  // serial voor de radio
void getGPS(){
  //Serial.println("GPS interupt");  // debug print
  while ((ss.available() > 0) && (!GPSUpdate)){
    int c = gps.encode(ss.read());
    if (c){
      //Serial.println("GPS read");  // debug print
      lat = gps.location.lat();
      lng = gps.location.lng();
      hour = gps.time.hour() + 1;
      min = gps.time.minute();
      sec = gps.time.second();
      /*radio.print(lat);
      radio.print("  ");
      radio.print(lng);
      radio.print("  ");
      radio.print(hour);
      radio.print("  ");
      radio.print(min);
      radio.print("  ");
      radio.print(sec);*/ // debug prints
      GPSUpdate = true;
    }
  }
  
 
}

void readSensors(){
  temp = bmp.readTemperature();
  press = bmp.readPressure();
  alt = bmp.readAltitude(1013.25);  // hier moet ook de meting van nu bij. (dat zou al in de setup kunnen worden gekalibreerd)

  CO = analogRead(A0) * (max_volts / max_analog_steps);
  NH3 = analogRead(A1) * (max_volts / max_analog_steps);
  NO2 = analogRead(A2) * (max_volts / max_analog_steps);

	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

  xacc = a.acceleration.x;
  yacc = a.acceleration.y;
  zacc = a.acceleration.z;
  totalAcceleration = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  Serial.print("xacc: ");
  Serial.println(xacc);
  Serial.print("yacc: ");
  Serial.println(yacc);
  Serial.print("zacc: ");
  Serial.println(zacc);
  Serial.print("Total Acceleration: ");
  Serial.println(totalAcceleration);
  if (totalAcceleration < 8) {
    Serial.println("Deploying parachute!");
    release.write(0);
    digitalWrite(13, HIGH);
    delay(1000);
  }  else {
    release.write(150);
  }
  digitalWrite(ledPower,LOW);  // vanaf hier pm
  delayMicroseconds(280);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(40);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(9680);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0){
    dustDensity = 0.00;
  }// tot hier pm

  O3 = analogRead(O3_pin) * (max_volts / max_analog_steps);
}

void radioSend(){
  // volgorde = lat, lng, hour, min, sec, co, nh3, no2, o3, pm, temp, press, alt
  radio.print(GPSUpdate);
  radio.print(", ");
  radio.print(lat, 6);
  radio.print(", ");
  radio.print(lng, 6);
  radio.print(", ");
  radio.print(hour);
  radio.print(", ");
  radio.print(min);
  radio.print(", ");
  radio.print(sec);
  radio.print(", ");
  radio.print(CO, 4);
  radio.print(", ");
  radio.print(NH3, 4);
  radio.print(", ");
  radio.print(NO2, 4);
  radio.print(",");
  radio.print(O3, 4);
  radio.print(", ");
  radio.print(dustDensity);
  radio.print(", ");
  radio.print(temp);
  radio.print(", ");
  radio.print(press);
  radio.print(", ");
  radio.println(alt);
  }

void setup() {
  //pinMode(RXPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(TXPin), getGPS, RISING);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);  // pins voor MICS sensor

  pinMode(ledPower, OUTPUT);  // pin voor PM sensor

  pinMode(O3_pin, INPUT);
  pinMode(7, INPUT);  // pin voor ozon
  release.attach(10);
  release.write(150);
  radio.begin(9600);  // start de radio!
  ss.begin(GPSBaud);  // om te communiceren met GPS

  Serial.begin(9600);
  

  while ( !Serial ) delay(100);   // wait for native usb

  Serial.println(F("CanSat code!"));
  //radio.println(F("CanSat code!"));  // eerste test print

  unsigned status;  // geen idee wat dit is
  status = bmp.begin();  // init voor de barometer
  /*if (!status) {  // debug voor barometer
    radio.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    radio.print("SensorID was: 0x");
    radio.println(bmp.sensorID(),16);
  } */

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    
  
}
bool checkAlt(){
  last_alt = new_alt;
  new_alt = accurateAlt();
  if ((last_alt - new_alt) < (-7)){return true;}  // EN als imu basicly 0 meet.
  else{return false;}
}
float accurateAlt(){
  const int n =5;
  float alt_gem;
  for (int i = 0; i>n; i++){
    alt_gem += bmp.readAltitude(1013.25);
  }
  alt_gem = alt_gem/n;
  return alt_gem;
}

void loop() {
  if(ss.available()){
    for (int i = 0; i<50; i++){  // Wat is de minimum? altitude moet ook gecheckt.
      getGPS();
      delay(10);
      if (GPSUpdate){break;}
    }
    GPSUpdate = false;
  } 
  if (!((ss.available() > 0))){
    GPSUpdate = false;
  }
  checkAlt();
  readSensors();
  radioSend();
}



