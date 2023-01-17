#include <SoftwareSerial.h>
const int ledPin = 13;
float pi = 3.14159265358979;
int dag = 365;
SoftwareSerial radio(8, 9);
void setup(){
  Serial.begin(9600);
  radio.begin(9600);
  pinMode(ledPin, OUTPUT);
}
 
void loop(){
  radio.print('<'); // start marker
  radio.print(dag);
  radio.print(','); // comma separator
  radio.print(pi, 6);
  radio.println('>'); // end marker
  delay(1000);
}
