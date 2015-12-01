String inputString = "";       
boolean stringComplete = false;  
int A = 0;
int B = 0;
int C = 0;



void setup() {
  Serial.begin(9600);   
  inputString.reserve(200);  
}

void loop() {
  serialEvent(); 
  if (stringComplete) {   
    Serial.println(inputString);
    if (inputString.substring(0, 2) == "FW") {
      A = inputString.substring(3,5).toInt(); 
      Serial.println("Edasi");
      Serial.println(A);
    }
    inputString = ""; 
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();     
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

