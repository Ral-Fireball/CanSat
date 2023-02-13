#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float x;
float y;
float z;
float totalAcceleration;

float totalA(float x, float y, float z){
  return sqrt(sq(x)+sq(y)+sq(z));
}

void setup(void) {
	Serial.begin(9600);

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
  if (totalAcceleration < 2) {
    //Serial.println("Deploying parachute!");
  }
	//Serial.println("");
	delay(50);
}