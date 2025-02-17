#include <Arduino.h>
#include "PinLayout.h"
#include <math.h>
#include <stdio.h>

#define LED1 PA3
#define LED2 PA1
#define LED3 PA0
#define TIRETTE PA5

enum state{
    DEBUT,
    TIRETTE_A_REMETTRE,
    PRET_DEPART,
    DEPART
};

state etat = DEBUT;


void setup (){
    Serial.begin(115200);
    pinMode(TIRETTE, INPUT_PULLUP);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
}


void loop(){;
    Serial.println((digitalRead(TIRETTE)));
    switch (etat)
    {
    case DEBUT:
        if (digitalRead(TIRETTE)==LOW){
            digitalWrite(LED1, HIGH);
            //digitalWrite(LED2, HIGH);
            //digitalWrite(LED3, HIGH);
            etat=TIRETTE_A_REMETTRE;
            delay(1000);
        }
        break;
    case TIRETTE_A_REMETTRE:
        if (digitalRead(TIRETTE)==HIGH){
            digitalWrite(LED1, LOW);
            digitalWrite(LED2, LOW);
            digitalWrite(LED3, LOW);
            etat=PRET_DEPART;
            delay(1000);
        }
        break;
    case PRET_DEPART:
        if (digitalRead(TIRETTE)==LOW){
            //delay(90000);
            //digitalWrite(LED1, HIGH);
            digitalWrite(LED2, HIGH);
            //digitalWrite(LED3, HIGH);
            etat=DEPART;
            delay(1000);
        }
        break;
    }

}
