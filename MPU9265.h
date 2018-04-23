#ifndef _MPU9265_H_
#define _MPU9265_H_

#include "AbstractSensor.h"

class MPU9265 : public AbstractSensor {
private :
    uint8_t address;

    void WriteByte(uint8_t reg, uint8_t data);
    uint8_t ReadByte(uint8_t reg);
    void ReadBytes(uint8_t reg, uint8_t count, uint8_t * dest);
    
    const char *accName;
    const char *gyrName;    
public :
    MPU9265(const char *aName, const char *gName);

    virtual void Init();
    virtual void Sense(OSCBundle *bundle);
};

#endif
