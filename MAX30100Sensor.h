#ifndef _MAX_30100_SENSOR_H_
#define _MAX_30100_SENSOR_H_

#include "AbstractSensor.h"

#include <MAX30100_PulseOximeter.h>

#define MAX30100_PULSE 0x01
#define MAX30100_BEAT  0x02

class MAX30100Sensor : public AbstractSensor {
private:
    const char *pName;
    const char *bName;

    uint8_t m;
    volatile bool beat;

    PulseOximeter pox;

    static void BeatCallback();

    MAX30100Sensor(const char *pulseName, const char *beatName, uint8_t mode);

    static MAX30100Sensor *instance;
public:
    static MAX30100Sensor *GetInstance(const char *pulseName, const char *beatName, uint8_t mode);

    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#endif
