#ifndef GRADIENT_H_
#define GRADIENT_H_

#include "led_strip.h"
#include <stdint.h>

class Gradient {
public:
    Gradient();

    struct Step {
        uint8_t pos;
        uint32_t color;
    };

    void clearSteps();
    void addStep(uint8_t pos, uint32_t color);
    void setSteps(Step *st, uint8_t steps);

    uint32_t getColor(uint8_t pos);

    /*  
    void smear();
    void randomize();
    void randomize(int, int); // set low and high range
    void peturb(int, int); 
    void fade();
    void wipe(byte level);
    */
    
private:
    Step m_steps[10];
    uint8_t m_stepCount;
};

#endif // GRADIENT_H_