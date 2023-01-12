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

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);  // voor prints, weet niet of dit uitendelijk nodig is

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);  // pins voor MICS sensor

  ss.begin(GPSBaud);  // om te communiceren met GPS

  while ( !Serial ) delay(100);   // wait for native usb

  Serial.println(F("CanSat code!"));  // eerste test print

  unsigned status;  // geen idee wat dit is
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();  // init voor de barometer
  if (!status) {  // debug voor barometer
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
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
  float temperature;
  float pressure;
  float altitude;  // variablen voor gemeten waarde barometer
  double lat;
  double lng;
  int hour;
  int min;
  int sec; // variabelen voor gemeten waarde GPS
  int CO;
  int NH3;
  int NO2;
}

void loop() {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25);  // hier moet ook de meting van nu bij. (dat zou al in de setup kunnen worden gekalibreerd)
    if (ss.available() > 0){
      gps.encode(ss.read());
      if (gps.location.isUpdated()){ 
        lat = gps.location.lat();
        lat = gps.location.lng();
        Serial.print(" Time: ");
        hour = gps.time.hour() + 1;
        minute = gps.time.minute();
        sec = gps.time.second();
      }
  }
  CO = analogRead(A0) * (max_volts / max_analog_steps);
  NH3 = analogRead(A1) * (max_volts / max_analog_steps);
  NO2 = analogRead(A2) * (max_volts / max_analog_steps);
  delay(2000);
}

// note: NO2, GPS en barometer gedaan
// TODO: onzon, fijnstof, telecommunicatie