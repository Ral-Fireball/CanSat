const int ledPin = 13;
const int switchPin = 2;
void setup()
{
  Serial.begin(9600);
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}
 
void loop(){
  Serial.write(HIGH);
  Serial.flush();
  delay(1000);
  Serial.write(LOW);
  Serial.flush();
  delay(1000);
}
