#ifndef _HCSR04_H_
#define _HCSR04_H_

#include "AbstractSensor.h"

template <uint8_t TRIG_PIN, uint8_t ECHO_PIN>
class HCSR04 : public AbstractSensor {
private:
    volatile unsigned long tStart;
    volatile float dist;
    volatile bool ready;
    static const char *senName;

    static HCSR04<TRIG_PIN, ECHO_PIN> *instance;

    static void Interrupt();
    
    void Start(unsigned long t);
    void Stop(unsigned long t);

    HCSR04();
public:
    static HCSR04<TRIG_PIN, ECHO_PIN> *GetInstance(const char *name);
    
    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#include "HCSR04.hpp"
#endif
