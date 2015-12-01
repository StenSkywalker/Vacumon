#include<Wire.h>
#include <SoftwareSerial.h>
int hPin1=5; //right backwards
int hPin2=6; //right forwards
int hPin3=9; //left forwards
int hPin4=10; //left backwards
int dgrees=1500;
int motorSpeed=1023;
int correctRight=0;
int correctLeft=22;
//Serial_IN
int i=0;
int a=0;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
//Mootor
bool L_on=0;
bool L_dir=0;
bool R_on=0;
bool R_dir=0;
int Time=0;  
//Pööramine
bool suund=0;
int nurk=0;
int full_circle=6280;
int one_degree=0;
int nurga_parandus=0;
// ultrasound
int trigPin =7;
int echoPin=8;
int duration, disctance, uDist; // Duration used to calculate distance
int obstacle;
//gyro variables 
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ;
int GryoAxis1 = A4;
int GryoAxis2 = A5;
int GryoAxis3 = 2;
String AxisX;
String AxisY;
String AxisZ;

#define Baudrate 9600
SoftwareSerial ESPserial(3, 4); // RX, TX

#define SSID "blackninja"
#define PASS "vacumon1"
#define IP "184.106.153.149" // ThingSpeak IP Address: 184.106.153.149
String GET = "GET /update?key=E9C2L5BF1BSSZQ2P";

void setup(){
  // initialize serials:
  Serial.begin(Baudrate);
  ESPserial.begin(Baudrate);

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  //motor
  pinMode(hPin1, OUTPUT);
  pinMode(hPin2, OUTPUT);
  pinMode(hPin3, OUTPUT);
  pinMode(hPin4, OUTPUT);
  //ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Gyro
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //WIFI Setup
  sendESPserial("AT");
  delay(2000);
  if(ESPserial.find("OK"))
    {
      int i=0;
    Serial.println("RECEIVED: OK\nData ready to sent!");  //added for debugging
    //ESPserial.println("RECEIVED: OK\nData ready to sent!");
    connectWiFi();  //function has 5 sec delay built in
    sendESPserial("AT+CWMODE=3");
    delay(500);
    sendESPserial("AT+CIPMUX=1");
    delay(500);
    sendESPserial("AT+CIPSERVER=1,8888");
    delay(500);
    sendESPserial("AT+CIFSR");
    while ( ESPserial.available())   {
      Serial.write( ESPserial.read());  
     
    }
    }
  else{
    Serial.println("nok to send");
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------MAIN----------------------------------------------------------------------------------------------
void loop() {
//  uDist=measDist();
//  if(uDist>10){

//  if ( Serial.available() )   {  ESPserial.write( Serial.read() );  }
//  if ( ESPserial.available() )   {  Serial.write( ESPserial.read() );  }
if (i==0){
  //delay(10000);
 Mootor(1, 1, 1, 1, 1000);
 Mootor(1, 0, 1, 1, 1000);
  serialEvent(); //call the function
    if (stringComplete && a>1) {
      Mootor(1, 1, 1, 0, 3250);
      Mootor(1, 1, 1, 1, 10000);
      Mootor(1, 0, 1, 1, 6500);
      Mootor(0, 0, 0, 0, 1000);
      i=1;
      Serial.println(inputString);
     /* if (Contains(inputString,"start")==1){
        ESPserial.println("Yes, Sir!");
        Mootor(1, 0, 1, 0, 1000);
        gryo();
        delay(1000);
        ESPserial.println("Done");
        Serial.println("Done");
        */
      }
     
      //ESPserial.println("WellDone");
      inputString = "";   // clear the string:
      stringComplete = false;
    } 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


bool Contains(String s, String search) {
    int max = s.length() - search.length();

    for (int i = 0; i <= max; i++) {
        if (s.substring(i) == search) return true; // or i
    }

    return false; //or -1
} 


//SerialEvent
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void serialEvent() {
  while (ESPserial.available()) {
    char inChar = (char)ESPserial.read();    // get the new byte:
    inputString += inChar;    // add it to the inputString:
    if (inChar == '\r') {
      stringComplete = true;
      a += 1;
    }
  }
}

//Mootori juhtimine    (X_on=0 => Motor OFF;   X_on=1 => Motor ON;   X_dir=0 => Run Backwards;   X_dir=1 => Run Forward;   Time => Motor activation time in ms;)
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Mootor(bool L_on, bool L_dir, bool R_on, bool R_dir, int Time){
  if(L_on==1){  //Turn on L motor
    if(L_dir==0){ //Direction BW
     analogWrite(hPin3,motorSpeed-correctLeft);
    }
    else{ //Direction FW
     analogWrite(hPin4,motorSpeed-correctLeft);
    }
  }
  if(R_on==1){  //Turn on R motor
    if(R_dir==0){ //Direction BW
     analogWrite(hPin2,motorSpeed-correctRight);
    }
    else{ //Direction FW
     analogWrite(hPin1,motorSpeed-correctRight);
    }
  }
  delay(Time);
  analogWrite(hPin1,0);
  analogWrite(hPin2,0);
  analogWrite(hPin3,0);
  analogWrite(hPin4,0);
}


//Mootori pööramise kalkulatsioon   (suund=0 => L; suund=1 => R; nurk=int => degrees)
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Turn(bool suund, int nurk){
  one_degree = full_circle / 360;
  int aeg = (nurk * one_degree) - nurga_parandus;
  if(suund==0){
    Mootor(1, 0, 1, 1, aeg);
  }
  else {
    Mootor(1, 1, 1, 0, aeg);
  }
}

//ULTRASOUND
//------------------------------------------------------------------------------------------------------------------------------------------------------
int measDist(){ // The following trigPin/echoPin cycle is used to determine the distance of the nearest object by bouncing soundwaves off of it.
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  
  disctance = duration/58.2;    //Calculate the distance (in cm) based on the speed of sound.
  return disctance;
  delay(50);    //Delay 50ms before next reading.
}

//GYRO
//------------------------------------------------------------------------------------------------------------------------------------------------------
void gryo(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.println(AcZ
  // value_temp = analogRead(sensor_temp);
  // Gryo end 
  
  // gryo data to string start 
  AxisX = String(AcX);
  AxisY = String(AcY);
  AxisZ = String(AcZ);
  updateTS(AxisX, AxisY, AxisZ);
}

//CINNECT WIFI
//------------------------------------------------------------------------------------------------------------------------------------------------------
boolean connectWiFi()
{
 ESPserial.println("AT+CWMODE=1");//WiFi STA mode - if '3' it is both client and AP
  delay(2000);
  //Connect to Router with AT+CWJAP="SSID","Password";
  // Check if connected with AT+CWJAP?
  String cmd="AT+CWJAP=\""; // Join accespoint
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  sendESPserial(cmd);
  delay(5000);
  if(ESPserial.find("OK"))
  {
    Serial.println("RECEIVED: OK");
    return true;
  }
  else
  {
    Serial.println("RECEIVED: Error ESPserial.find_OK");
    return false;
  }

  cmd = "AT+CIPMUX=0";// Set Single connection
  sendESPserial( cmd );
  if( ESPserial.find( "Error") )
  {
    Serial.print( "RECEIVED: Error AT+CIPMUX=0" );
    return false;
  }
}


// Send to ESPserail
//------------------------------------------------------------------------------------------------------------------------------------------------------
void sendESPserial(String cmd) 
{
  ESPserial.print("SEND: ");
  Serial.print("SEND: ");
  ESPserial.println(cmd);
  Serial.println(cmd);
}

//UPDATE TS
//------------------------------------------------------------------------------------------------------------------------------------------------------
void updateTS( String T, String tM, String H)
{
  ESPserial.println(T + " " + tM + " " + H);
  // ESP8266 Client
  String cmd = "AT+CIPSTART=\"TCP\",\"";// Setup TCP connection
  cmd += IP;
  cmd += "\",80";
  sendESPserial(cmd);
  delay(2000);
  
  if( ESPserial.find( "Error" ) )
  {
    Serial.print( "RECEIVED: Error\nExit1" );
    return;
  }
  
cmd = GET + "&field1=" + T + "&field2=" + tM + "&field3=" + H +"\r\n";
//  cmd = GET + "&field1=1\r\n";
//  cmd = GET + "&field1=" + T + "&field2=" + tM + "&field3=" + H +0x0D+0x0A;
  Serial.print( "AT+CIPSEND=" );
  ESPserial.print( "AT+CIPSEND=" );
  Serial.println( cmd.length() );
  ESPserial.println( cmd.length() );

  if(ESPserial.find(">") )
  {
    Serial.println(">");  //debugging
    ESPserial.print(cmd);
    Serial.println("message sent");
  }
  else
  {
    sendESPserial( "AT+CIPCLOSE" );//close TCP connection
  }

  if( ESPserial.find("OK") )
  { 
    Serial.println( "RECEIVED: OK" );
    ESPserial.println( "RECEIVED: OK" );
  }
  else
  {
    Serial.println( "RECEIVED: Error\nExit2" );
    ESPserial.println( "RECEIVED: Error\nExit2" );
  }
}

