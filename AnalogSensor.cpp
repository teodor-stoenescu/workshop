#include "AnalogSensor.h"

AnalogSensor::AnalogSensor(const char *name, int input, int sampleWindow) {
    senName = name;
    pin = input;
    sampleWin = sampleWindow;
}

void AnalogSensor::Init() {

}

void AnalogSensor::Sense(OSCBundle *bundle) {
    unsigned long stopMillis = millis() + sampleWin;  // Start of sample window
    unsigned int peakToPeak = 0;   // peak-to-peak level
    
    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;
    unsigned int count = 0;

    unsigned int sample;
    
    // collect data for 50 mS
    while (millis() < stopMillis) {
        sample = analogRead(pin);
        if (sample < 1024)  // toss out spurious readings
        {
            count++;
            if (sample > signalMax)
            {
                signalMax = sample;  // save just the max levels
            }
            if (sample < signalMin)
            {
                signalMin = sample;  // save just the min levels
            }
        }
    }

    peakToPeak = signalMax - signalMin;

    //bundle->add(senName).add(peakToPeak);
    bundle->add(senName).add((signalMax + signalMin) / 2);
    Serial.print("MIC: "); Serial.print(sample); Serial.print(" "); Serial.print(count); Serial.print(" "); Serial.print(signalMax); Serial.print(" "); Serial.println(signalMin);
}
