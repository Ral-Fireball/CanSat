const float max_volts = 5.0;
const float max_analog_steps = 1023.0;

void setup() 
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setup initializing");

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}



void loop() 
{
  int a0_read = analogRead(A0);
  int a1_read = analogRead(A1);
  int a2_read = analogRead(A2);
  Serial.print("CO (a0): ");
  Serial.println(a0_read * (max_volts / (max_analog_steps * 80)));
  Serial.print("NH3 (a1): ");
  Serial.println(a1_read * (max_volts / (max_analog_steps * 60)));
  Serial.print("NO2 (a2): ");
  Serial.println(a2_read* (max_volts / (max_analog_steps * 80)));
  

  delay(500);
}