#ifndef _MPR121_H_
#define _MPR121_H_

#include <Adafruit_MPR121.h>

#include "AbstractSensor.h"

#define MPR121_BUTTONS 12

class MPR121 : public AbstractSensor {
private:
    Adafruit_MPR121 sen;

    const char *names[MPR121_BUTTONS];
    bool ready;
public:
    MPR121(const char *buttons[MPR121_BUTTONS]);

    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#endif
