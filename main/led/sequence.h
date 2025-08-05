#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include "../packet/packet.h"
#include "pattern.h"
#include <stdint.h>

class Sequence {
public:
    enum Type {
        IDLE,
        ALERT,
        RANDOM,
        NONE,
    };

    virtual ~Sequence() = default;

    // Start at the first set of steps
    virtual int Reset() { return 0; }
    //! Advance to the next step
    virtual int Advance(int step, bool timed = false) { return (step + 1) % GetStepCount(); }
    
    //! Number of steps
    virtual int GetStepCount() const = 0;
    
    //! Time (in ms) to stay in this step
    virtual uint32_t GetDuration(int step) const = 0;

    virtual int GetCommand(int step) const { return 2; } // HC_PATTERN
    virtual int GetBrightness(int step) const = 0;
    virtual int GetSpeed(int step) const = 0;
    virtual int GetPatternId(int step) const = 0;
    virtual uint32_t GetColors(int step, int color) const = 0;
    virtual uint8_t GetLevels(int step, int level) const = 0;
};

class PacketSequence : public Sequence {
public:
    explicit PacketSequence(Packet *_packet) : packet(_packet) { }

    virtual int GetStepCount() const override { return 1; }
    virtual uint32_t GetDuration(int step) const override { return 0; }
    virtual int GetCommand(int step) const override { return packet->command; }
    virtual int GetBrightness(int step) const override { return packet->brightness; }
    virtual int GetSpeed(int step) const override { return packet->speed; }
    virtual int GetPatternId(int step) const override { return packet->pattern; }
    virtual uint32_t GetColors(int step, int color) const override { return packet->color[color]; }
    virtual uint8_t GetLevels(int step, int level) const override { return packet->level[level]; }

    Packet *packet;
};

#pragma pack(push, 1)
struct Step {
    uint32_t duration;
    uint8_t brightness;
    uint8_t speed;
    uint8_t pattern;
    uint32_t colors[3];
    uint8_t level;
};
#pragma pack(pop)

class StepSequence : public Sequence {
public:
    StepSequence(const Step *_steps, int _stepCount) : steps(_steps), stepCount(_stepCount) { }
    virtual int GetStepCount() const override { return stepCount; }
    virtual uint32_t GetDuration(int step) const override { return steps[step].duration; }
    virtual int GetBrightness(int step) const override { return steps[step].brightness; }
    virtual int GetSpeed(int step) const override { return steps[step].speed; }
    virtual int GetPatternId(int step) const override { return steps[step].pattern; }
    virtual uint32_t GetColors(int step, int color) const override { return steps[step].colors[color]; }
    virtual uint8_t GetLevels(int step, int level) const override { return (level == 0) ? steps[step].level : 0; }

    const Step *steps;
    int stepCount;
};

class IdleSequence : public StepSequence {
public:
    IdleSequence();
};

class AlertSequence : public StepSequence {
public:
    AlertSequence();
};

class RandomSequence : public StepSequence {
public:
    RandomSequence();
    
    virtual int Reset() override;
    virtual int Advance(int step, bool timed = false) override;
};

#endif // SEQUENCE_H_