#ifndef _HCSR04_HPP_
#define _HCSR04_HPP_

#include "HCSR04.h"

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
HCSR04<TRIG_PIN, ECHO_PIN> *HCSR04<TRIG_PIN, ECHO_PIN>::instance = nullptr;

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
const char *HCSR04<TRIG_PIN, ECHO_PIN>::senName = nullptr;

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
void HCSR04<TRIG_PIN, ECHO_PIN>::Interrupt() {
    
    if (nullptr != instance) {
        unsigned long t = micros();

        if (digitalRead(ECHO_PIN) == HIGH) {
            instance->Start(t);
        } else {
            instance->Stop(t);
        }
    }
}

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
void HCSR04<TRIG_PIN, ECHO_PIN>::Start(unsigned long t) {
    tStart = t;
}


template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
void HCSR04<TRIG_PIN, ECHO_PIN>::Stop(unsigned long t) {
    dist = (t - tStart) / 58.0f;
    tStart = t;
    ready = true;
}


template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
HCSR04<TRIG_PIN, ECHO_PIN>::HCSR04() { 
    dist = 0.0;
    ready = true;
}

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
HCSR04<TRIG_PIN, ECHO_PIN> *HCSR04<TRIG_PIN, ECHO_PIN>::GetInstance(const char *name) {
    static HCSR04<TRIG_PIN, ECHO_PIN> inst;

    instance = &inst;

    senName = name;
    return instance;
}

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
void HCSR04<TRIG_PIN, ECHO_PIN>::Init() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), HCSR04<TRIG_PIN, ECHO_PIN>::Interrupt, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(ECHO_PIN), HCSR04<TRIG_PIN, ECHO_PIN>::InterruptRising, RISING);
    
}

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
void HCSR04<TRIG_PIN, ECHO_PIN>::Sense(OSCBundle *bundle) {
    float localDist = dist;
    bundle->add(senName).add(localDist);

    if (ready) {
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        ready = false;
    }

    //Serial.print("Dist: "); Serial.println(localDist);
    //Serial.print("TStart: "); Serial.println(tStart);
}

#endif
