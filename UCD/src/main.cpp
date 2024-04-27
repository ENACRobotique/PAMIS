#include <Arduino.h>
#include "PinLayout.h"
#include <math.h>
#include <stdio.h>

#define LED PA3
#define TIRETTE PA5

void setup (){
    Serial.begin(115200);
    pinMode(TIRETTE, OUTPUT);
    pinMode(LED, OUTPUT);
}


void loop(){
    digitalWrite(TIRETTE, HIGH);
    if (digitalRead(TIRETTE)== LOW){
        delay(90000);
        digitalWrite(LED, HIGH);
    }
}
