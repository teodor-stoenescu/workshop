#include "MPR121.h"

#define MPR121_I2C_ADDR 0x5B

MPR121::MPR121(const char *buttons[MPR121_BUTTONS]) : sen(Adafruit_MPR121()) {
    for (int i = 0; i < MPR121_BUTTONS; ++i) {
        names[i] = buttons[i];
    }
}

void MPR121::Init() {
    if (!sen.begin(MPR121_I2C_ADDR)) {
        return;
    }

    ready = true;
}

void MPR121::Sense(OSCBundle *bundle) {
    if (!ready) {
        return;
    }

    uint16_t touched = sen.touched();
    for (int i = 0; i < MPR121_BUTTONS; ++i) {
        if (nullptr != names[i]) {
            bundle->add(names[i]).add((touched >> i) & 1);
        }
    }
}
