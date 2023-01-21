#include <Adafruit_BMP280.h>  // voor de barometer
#include <Wire.h>  // voor basicly alles, komt uit barometer
#include <SPI.h>  // barometer
#include <SoftwareSerial.h>  // komt uit gps
#include <TinyGPS++.h>  // voor gps

Adafruit_BMP280 bmp; // I2C

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

SoftwareSerial radio(8, 9);  // serial voor de radio

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);  // pins voor MICS sensor

  pinMode(ledPower, OUTPUT);  // pin voor PM sensor

  pinMode(A0, INPUT);
  pinMode(7, INPUT);  // pin voor ozon

  radio.begin(9600);  // start de radio!
  ss.begin(GPSBaud);  // om te communiceren met GPS

  Serial.begin(9600);
  

  while ( !Serial ) delay(100);   // wait for native usb

  Serial.println(F("CanSat code!"));
  radio.println(F("CanSat code!"));  // eerste test print

  unsigned status;  // geen idee wat dit is
  status = bmp.begin();  // init voor de barometer
  if (!status) {  // debug voor barometer
    radio.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    radio.print("SensorID was: 0x");
    radio.println(bmp.sensorID(),16);
  } 

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
}

void loop() {
   
  while ((ss.available() > 0)){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){ 
      lat = gps.location.lat();
      lng = gps.location.lng();
      hour = gps.time.hour() + 1;
      min = gps.time.minute();
      sec = gps.time.second();
      } else{continue;}
    

    Serial.print("test");
    readSensors();
    radioSend();
  }
  if (!((ss.available() > 0))){
    radio.println("GEEN GPS");
    readSensors();
    radioSend();
  }
}

void readSensors(){
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

  if ( dustDensity < 0){
    dustDensity = 0.00;
  }// tot hier pm

  O3 = analogRead(O3_pin) * (max_volts / max_analog_steps);
}

void radioSend(){
  radio.print("O3 concentration (ppm): ");
  radio.println(O3);
  radio.print(lat, 6);
  radio.print(" lng: ");
  radio.print(lng, 6);
  radio.print(" H; ");
  radio.print(hour);
  radio.print(" m: ");
  radio.print(min);
  radio.print(" s: ");
  radio.print(sec);
  radio.print(" co: ");
  radio.print(CO, 4);
  radio.print(" nh3: ");
  radio.print(NH3, 4);
  radio.print(" no2: ");
  radio.print(NO2, 4);
  radio.print(" o3: ");
  radio.print(O3, 4);
  radio.print(" pm: ");
  radio.println(dustDensity);
  radio.print(" temp: ");
  radio.print(temp);
  radio.print(" press: ");
  radio.print(press);
  radio.print(" alt: ");
  radio.println(alt);
}
