#include "gradient.h"
#include <cstring>

Gradient::Gradient() : m_stepCount(0) {
    memset(m_steps, 0, sizeof(m_steps));
}

void Gradient::clearSteps() {
    m_stepCount = 0;
    memset(m_steps, 0, sizeof(m_steps));
}

void Gradient::addStep(uint8_t pos, uint32_t color) {
    if (m_stepCount < 10) {
        m_steps[m_stepCount].pos = pos;
        m_steps[m_stepCount].color = color;
        m_stepCount++;
    }
}

void Gradient::setSteps(Step *st, uint8_t steps) {
    m_stepCount = (steps > 10) ? 10 : steps;
    memcpy(m_steps, st, m_stepCount * sizeof(Step));
}

uint32_t Gradient::getColor(uint8_t pos) {
    if (m_stepCount == 0) {
        return 0;
    }
    
    // Use Arduino's exact gradient logic - check boundaries first
    if (pos <= m_steps[0].pos) {
        return m_steps[0].color;
    } else if (pos >= m_steps[m_stepCount - 1].pos) {
        return m_steps[m_stepCount - 1].color;
    } else {
        // Find the interpolation range
        int i = 0;
        while (i < (m_stepCount - 1) && 
               !(pos >= m_steps[i].pos && pos < m_steps[i + 1].pos)) {
            i++;
        }
        if (i >= (m_stepCount - 1)) {
            return 0;
        }
        if (m_steps[i + 1].pos == m_steps[i].pos) {
            return m_steps[i].color;
        }
        
        // Arduino's exact interpolation formula
        uint8_t f = (pos - m_steps[i].pos) * 255 / 
                    (m_steps[i + 1].pos - m_steps[i].pos);
        return LEDStrip::ColorBlend(m_steps[i].color, m_steps[i + 1].color, f);
    }
}