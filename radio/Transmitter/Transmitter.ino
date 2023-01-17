const int ledPin = 13;
const int switchPin = 2;
 
int switchState = 0;
 
void setup()
{
  Serial.begin(9600);
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}
 
void loop()
{   
    switchState = digitalRead(switchPin);
 
    if(switchState == HIGH)
      {
      Serial.write(switchState);
      Serial.flush();
      digitalWrite(ledPin, HIGH);
      Serial.println(switchState); 
      delay(1); 
      }
    else
      {
      Serial.write(switchState);
      Serial.flush();
      digitalWrite(ledPin, LOW);
      Serial.println(switchState);
      delay(1);
      }
}