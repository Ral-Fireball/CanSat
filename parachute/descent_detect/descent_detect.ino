#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Servo release;

Adafruit_BMP280 bmp; // I2C
float last_alt;
float new_alt;
float dalt;

Adafruit_MPU6050 mpu;

float x;
float y;
float z;
float totalAcceleration;



float accurateAlt(){
  const int n = 10;
  float alt_gem;
  for (int i = 0; i<n; i++){
    alt_gem = bmp.readAltitude(1013.25);
  }
  alt_gem = alt_gem/n;
  return alt_gem;
}

float totalA(float x, float y, float z){
  return sqrt(sq(x)+sq(y)+sq(z));
}

void deploy(){
  release.write(0);
}

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
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
    while(1){delay(1);}
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
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  totalAcceleration = totalA(x, y, z);
  Serial.println(totalAcceleration);
	delay(50);

  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.print(x);
  Serial.print("/");
  Serial.print(y);
  Serial.print("/");
  Serial.println(z);

  last_alt = new_alt;
  new_alt = accurateAlt();
  dalt = last_alt - new_alt;

  
  
  if ((dalt > 0) && (totalAcceleration < 2)){
    Serial.println("parachute deploy!");
    digitalWrite(13, HIGH);
    deploy();
    delay(1000);
  }else{digitalWrite(13, LOW); release.write(150);}
  delay(1000);
}


