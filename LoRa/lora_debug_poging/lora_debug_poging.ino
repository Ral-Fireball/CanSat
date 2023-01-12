#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);

}

void loop(){
  if (mySerial.available())
    Serial.write(mySerial.read());
    
  if (Serial.available())
  { 
    while(Serial.available())
    {
      mySerial.write(Serial.read());
    }
    mySerial.println();
  }
  Serial.println(command("AT\r\n", 100));
}
String command(const char *toSend, unsigned long milliseconds) {
  String result;
  Serial.print("Sending: ");
  Serial.println(toSend);
  mySerial.println(toSend);
  unsigned long startTime = millis();
  Serial.print("Received: ");
  while (millis() - startTime < milliseconds) {
    if (mySerial.available()) {
      char c = mySerial.read();
      Serial.write(c);
      result += c;  // append to the result string
    }
  }
Serial.println();  // new line after timeout.
return result;
}