#pragma once
#include <stdint.h>     // uint32_t — compatible AVR/ESP/ARM
#include <Arduino.h>

// ============================================================
//  Timer  
// ============================================================
class Timer {
public:
    explicit Timer();
    explicit Timer(uint32_t period_us, bool active = false,
                   uint32_t startTime = 0);

    void     startTimer(uint32_t us = 1000000);
    void     restart();
    void     setTimer(uint32_t us);
    void     accountPeriod();
    void     stop();
    bool     expired()   const;
    uint32_t elapsed()   const;
    uint32_t remaining() const;
    bool     active()    const;
    uint32_t getPeriod() const;

private:
    uint32_t period  { 0     };
    uint32_t start   { 0     };
    bool     running { false };
};