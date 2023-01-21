#include <Adafruit_BMP280.h>
#include <Wire.h>

Adafruit_BMP280 bmp; // I2C

void setup() {
  Serial.begin(9600);
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
    Serial.println(calcV());
    float verticalV = calcV();
    if ((verticalV < 7) & (verticalV > -7)){
      Serial.println("apogee");
    }
    delay(2000);
}

float calcV(){
  float alt1 = bmp.readAltitude(1013.25);
  delay(100);
  float alt2 = bmp.readAltitude(1013.25);
  return (alt1-alt2)*10;
}
