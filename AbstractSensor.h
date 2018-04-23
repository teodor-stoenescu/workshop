#ifndef _ABSTRACT_SENSOR_H_
#define _ABSTRACT_SENSOR_H_

#include <OSCBundle.h>

class AbstractSensor {
public :
    virtual void Init() = 0;
    virtual void Sense(OSCBundle *bundle) = 0;
};

#endif
