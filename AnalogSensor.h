#ifndef _ANALOG_SENSOR_H_
#define _ANALOG_SENSOR_H_

#include "AbstractSensor.h"

class AnalogSensor : public AbstractSensor {
private:
    const char *senName;
    int pin;
    int sampleWin;
public:
    AnalogSensor(const char *name, int input, int sampleWindow);

    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#endif
