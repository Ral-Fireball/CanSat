 #include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX
void setup()
{
  Serial.begin(9600);
  pinMode(7,OUTPUT);
  mySerial.begin(9600);
}
 
void loop(){
  int Data;
  if(mySerial.available()>0){
    Data = mySerial.read();
    Serial.println(Data);
    mySerial.flush();
    }
  if (Data==1){
    digitalWrite(7, HIGH);
  }else{
    digitalWrite(7, LOW);
  }
  delay(1);  
}
 
void lowvalue()
  {
  digitalWrite(7, LOW);
  mySerial.println("0");
  }
 
 
void highvalue()
  {
  digitalWrite(7, HIGH);
  mySerial.println("1");
  }
