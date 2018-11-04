# define GATE 13
# define LED 13

void setup() {
  // set pwm pin
  pinMode(GATE, OUTPUT);
//  pinMode(LED, OUTPUT);
  // make sure prop starts off
  //digitalWrite(GATE, LOW);

}

void loop() {
  /*
  analogWrite(GATE, 255);
  //digitalWrite(LED, HIGH);
  delay(15);
  analogWrite(GATE, 100);
  //digitalWrite(LED, LOW);
  delay(15);
  */
  for(int i = 0; i < 255; i++){
    analogWrite(GATE, i);
    delay(15);
  }
  for(int i = 255; i >= 0; i--){
    analogWrite(GATE, i);
    delay(15);
  }
}

/*
#define fadePin 3

void setup(){
pinMode(fadePin, OUTPUT);
}

void loop(){

for(int i = 0; i<360; i++){ 
  //convert 0-360 angle to radian (needed for sin function) 
  float rad = DEG_TO_RAD * i; 
  //calculate sin of angle as number between 0 and 255 
  int sinOut = constrain((sin(rad) * 128) + 128, 0, 255); 
  analogWrite(fadePin, sinOut); 
  delay(15); 
  } 
 }
 */
