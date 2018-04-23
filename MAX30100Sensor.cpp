#include "MAX30100Sensor.h"

MAX30100Sensor *MAX30100Sensor::instance = nullptr;

void MAX30100Sensor::BeatCallback() {
    instance->beat = true;
}

MAX30100Sensor::MAX30100Sensor(const char *pulseName, const char *beatName, uint8_t mode) {
    pName = pulseName;
    bName = beatName;

    m = mode;
}

MAX30100Sensor *MAX30100Sensor::GetInstance(const char *pulseName, const char *beatName, uint8_t mode) {
    static MAX30100Sensor inst(pulseName, beatName, mode);

    instance = &inst;
    return instance;
}

void MAX30100Sensor::Init() {
    if (!pox.begin()) {
        Serial.println("POX not ready!!!");
        delay(50000);
    }

    if (m & MAX30100_BEAT) {
        pox.setOnBeatDetectedCallback(BeatCallback);
    }
}

void MAX30100Sensor::Sense(OSCBundle *bundle) {
    pox.update();
    
    if (m & MAX30100_PULSE) {
        float heartRate = pox.getHeartRate();
        uint8_t spO2 = pox.getSpO2();
    
        bundle->add(pName).add(heartRate).add(spO2);
        Serial.print("Heartrate: "); Serial.print(heartRate); Serial.print(" SpO2: "); Serial.println(spO2);
    }

    if (m & MAX30100_BEAT) {
        if (beat) {
            beat = false;
            bundle->add(bName).add(1);
            Serial.println("Beat ");
        }
    }
}

