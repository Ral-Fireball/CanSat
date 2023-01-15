#include <Adafruit_BMP280.h>  // voor de barometer
#include <Wire.h>  // voor basicly alles, komt uit barometer
#include <SPI.h>  // barometer
#include <SoftwareSerial.h>  // komt uit gps
#include <TinyGPS++.h>  // voor gps

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

const float max_volts = 5.0;
const float max_analog_steps = 1023.0;  // constantes voor analoge inputs

float temp;
float press;
float alt;  // variablen voor gemeten waarde barometer

double lat;
double lng;
int hour;
int min;
int sec; // variabelen voor gemeten waarde GPS

float CO;
float NH3;
float NO2;  // variabelen voor de gas sensor

int measurePin = A5; //de zwarte kabel moet in A5
int ledPower = 7; //witte kabel moet in gpio ... (in dit geval 7)
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;  // variabelen voor de pm sensor

int O3_pin = A3;
float O3;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);  // voor prints, weet niet of dit uitendelijk nodig is

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);  // pins voor MICS sensor

  pinMode(ledPower, OUTPUT);  // pin voor PM sensor

  pinMode(A0, INPUT);
  pinMode(7, INPUT);  // pin voor ozon

  ss.begin(GPSBaud);  // om te communiceren met GPS

  while ( !Serial ) delay(100);   // wait for native usb

  Serial.println(F("CanSat code!"));  // eerste test print

  unsigned status;  // geen idee wat dit is
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();  // init voor de barometer
  if (!status) {  // debug voor barometer
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  } 

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
}

void loop() {
   
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){ 
      lat = gps.location.lat();
      lng = gps.location.lng();
      Serial.print(" Time: ");
      hour = gps.time.hour() + 1;
      min = gps.time.minute();
      sec = gps.time.second();
      } else{continue;}
      temp = bmp.readTemperature();
    press = bmp.readPressure();
    alt = bmp.readAltitude(1013.25);  // hier moet ook de meting van nu bij. (dat zou al in de setup kunnen worden gekalibreerd)

    CO = analogRead(A0) * (max_volts / max_analog_steps);
  NH3 = analogRead(A1) * (max_volts / max_analog_steps);
  NO2 = analogRead(A2) * (max_volts / max_analog_steps);

  digitalWrite(ledPower,LOW);  // vanaf hier pm
  delayMicroseconds(280);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(40);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(9680);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }// tot hier pm

  O3 = analogRead(O3_pin) * (max_volts / max_analog_steps);
  Serial.print("O3 concentration (ppm): ");
  Serial.println(O3);
  Serial.print(lat, 6);
  Serial.print(" lng: ");
  Serial.print(lng, 6);
  Serial.print(" H; ");
  Serial.print(hour);
  Serial.print(" m: ");
  Serial.print(min);
  Serial.print(" s: ");
  Serial.print(sec);
  Serial.print(" co: ");
  Serial.print(CO, 4);
  Serial.print(" nh3: ");
  Serial.print(NH3, 4);
  Serial.print(" no2: ");
  Serial.print(NO2, 4);
  Serial.print(" o3: ");
  Serial.print(O3, 4);
  Serial.print(" pm: ");
  Serial.println(dustDensity);
  Serial.print(" temp: ");
  Serial.print(temp);
  Serial.print(" press: ");
  Serial.print(press);
  Serial.print(" alt: ");
  Serial.println(alt);
  }
  


  
}

// note: NO2, GPS, fijnstof en barometer gedaan
// werkend inc wire: NO2, Fijnstof
// TODO: onzon, telecommunicatie
