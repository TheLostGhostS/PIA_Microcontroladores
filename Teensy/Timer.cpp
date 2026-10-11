#include "Timer.hpp"

// ----------------------------------------------------------------
//  Timer
// ----------------------------------------------------------------
Timer::Timer() : period(0), start(0), running(false) {}

Timer::Timer(uint32_t period_us, bool active, uint32_t startTime)
    : period(period_us), start(startTime), running(active)
{
    // Si active=true y startTime=0, el timer arranca desde el boot.
    // Para arrancar desde ahora, llamar restart() después de construir,
    // o usar startTimer(us) que llama restart() internamente.
    if (period == 0) running = false;
}

void Timer::startTimer(uint32_t us) { setTimer(us); restart(); }

void Timer::restart() {
    if (period == 0) return;
    start   = micros();
    running = true;
}

void Timer::setTimer(uint32_t us) {
    if (us == 0) running = false;
    period = us;
}

void Timer::accountPeriod() {
    if (!running || period == 0) return;
    while(expired())
        start += period;   
}

void     Timer::stop()            { running = false; }
bool     Timer::expired()   const { return running && (micros() - start) >= period; }
uint32_t Timer::elapsed()   const { return running ? (micros() - start) : 0u; }
bool     Timer::active()    const { return running; }
uint32_t Timer::getPeriod() const { return period; }

uint32_t Timer::remaining() const {
    if (!running) return 0u;
    uint32_t e = elapsed();
    return (e >= period) ? 0u : (period - e);
}