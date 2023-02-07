#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_BMP280 bmp; // I2C
float last_alt;
float new_alt;
float dalt;

Servo release;

float accurateAlt(){
  const int n = 10;
  float alt_gem;
  Serial.println("acc");
  for (int i = 0; i<n; i++){
    alt_gem = bmp.readAltitude(1013.25);
  }
  alt_gem = alt_gem/n;
  return alt_gem;
}

void deploy(){
  release.write(0);
}

void setup() {
  Serial.begin(9600);

  pinMode(11, OUTPUT);
  release.attach(10);
  release.write(0);

  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {

  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
  last_alt = new_alt;
  new_alt = accurateAlt();
  Serial.println(new_alt);
  dalt = last_alt - new_alt;
  if (dalt > 0){
    Serial.println("parachute deploy!");
    digitalWrite(11, HIGH);
    deploy();
  }else{digitalWrite(11, LOW); release.write(150);}
  delay(1000);
}


