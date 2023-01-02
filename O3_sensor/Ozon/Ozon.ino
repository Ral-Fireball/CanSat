const float max_volts = 5.00;
const float max_analog_steps = 1023.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("starting test");
  pinMode(A0, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x = analogRead(A0);
  int y = digitalRead(7);
  Serial.print("O3 concentration (ppm): ");
  Serial.println(x * (max_volts / max_analog_steps));
  Serial.println(y);
  delay(2000);
}
