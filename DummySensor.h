#ifndef _DUMMY_SENSOR_H_
#define _DUMMY_SENSOR_H_

#include "AbstractSensor.h"

class DummySensor : public AbstractSensor {
public :
    virtual void Init() {
    }

    virtual void Sense(OSCBundle *bundle) {
        bundle->add("/test").add(1).add(0.5f);
    }
};

#endif
