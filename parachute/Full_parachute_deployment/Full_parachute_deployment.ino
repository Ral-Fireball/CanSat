#include <Adafruit_BMP280.h>

#include <Servo.h>

Servo release;
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float x;
float y;
float z;
float totalAcceleration;

Adafruit_BMP280 bmp; // I2C
float last_alt;
float new_alt;
float dalt;

float totalA(float x, float y, float z){
  return sqrt(sq(x)+sq(y)+sq(z));
}

void deploy(){
  release.write(0);
}

void setup(void) {
	Serial.begin(9600);

  pinMode(13, OUTPUT);
  release.attach(10);
  delay(1000);
  // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    while(1){delay(1);}
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);

		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  release.write(150);
	delay(100);
}

void loop() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
	/* Print out the valuves */
	/*Serial.print("Acceleration X: ");
	Serial.print(x);
	Serial.print(", Y: ");
	Serial.print(y);
	Serial.print(", Z: ");
	Serial.print(z);
	Serial.println(" m/s^2");*/

  totalAcceleration = totalA(x, y, z);
  Serial.println(totalAcceleration);
  
	//Serial.println("");
  //Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  //Serial.println(" m");

  last_alt = new_alt;
  new_alt = bmp.readAltitude(1013.25);
  dalt = last_alt - new_alt;

  if (totalAcceleration < 2 & dalt > -0.3) {
    //Serial.println("Deploying parachute!");
    digitalWrite(13, HIGH);
    deploy();
  }else{
    digitalWrite(13, LOW);
    }
	delay(50);
}