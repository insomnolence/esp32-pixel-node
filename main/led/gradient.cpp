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
    
    if (m_stepCount == 1) {
        return m_steps[0].color;
    }
    
    // Find the two steps to interpolate between
    for (uint8_t i = 0; i < m_stepCount - 1; i++) {
        if (pos >= m_steps[i].pos && pos <= m_steps[i + 1].pos) {
            // Interpolate between steps[i] and steps[i+1]
            uint8_t range = m_steps[i + 1].pos - m_steps[i].pos;
            if (range == 0) {
                return m_steps[i].color;
            }
            
            uint8_t offset = pos - m_steps[i].pos;
            uint8_t blend = (255 * offset) / range;
            
            return LEDStrip::ColorBlend(m_steps[i].color, m_steps[i + 1].color, blend);
        }
    }
    
    // If we get here, pos is outside our range
    if (pos < m_steps[0].pos) {
        return m_steps[0].color;
    } else {
        return m_steps[m_stepCount - 1].color;
    }
}