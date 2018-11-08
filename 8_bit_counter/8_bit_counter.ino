#include "SevSeg.h"
SevSeg sevseg; 

void setup(){
    byte numDigits = 1;
    byte digitPins[] = {};
    byte segmentPins[] = {10,9,5,3,2,11,12,6};
    bool resistorsOnSegments = true;

    byte hardwareConfig = COMMON_CATHODE; 
    sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
    sevseg.setBrightness(50);
}

void loop(){
        for (int i = 0; i < 10; i++){
          sevseg.setNumber(i);
          delay(3000);
          sevseg.refreshDisplay();
        }
        sevseg.refreshDisplay();        
}
