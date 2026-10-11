#include "WString.h"
#include <limits.h>
#include <math.h>       // sqrtf, fabsf — compatible AVR/ESP/ARM
#include <stdlib.h>     // labs() — compatible AVR/ESP/ARM
#include "Actuators.hpp"

// ================================================================
//
//   IMPLEMENTACIONES
//
// ================================================================


// ================================================================
//   Stepper
// ================================================================
StepperMotor::StepperMotor(int8_t stepPin, int8_t dirPin, int8_t enablePin,
                                   bool invert, uint32_t stepsPerRev)
    : stepPin_    { stepPin    }
    , dirPin_     { dirPin     }
    , enablePin_  { enablePin  }
    , inverted_   { invert     }
    , stepsPerRev_{ stepsPerRev}
    , tolerance_  { (float)stepsPerRev * 0.25f }
{}

bool StepperMotor::begin() {
    if (initialized_)      return false;   // solo una vez
    if (stepsPerRev_ == 0) return false;
    if (stepPin_   != -1) { pinMode(stepPin_,   OUTPUT); digitalWrite(stepPin_,   LOW); }
    if (dirPin_    != -1) { pinMode(dirPin_,    OUTPUT); }
    if (enablePin_ != -1) { pinMode(enablePin_, OUTPUT); digitalWrite(enablePin_, LOW); }
    c0_ = 0.676f * sqrtf(2.0f / accel_) * 1000000.0f;
    if (c0_ < (float)(MIN_STEP_PERIOD_US * 2)) c0_ = (float)(MIN_STEP_PERIOD_US * 2);
    initialized_ = true;
    return true;
}

// ---- Getters ---------------------------------------------------
uint32_t StepperMotor::getStepsPerRev()  const { return stepsPerRev_;  }
int32_t  StepperMotor::getCurrentStep()  const { return currentStep_;  }
int32_t  StepperMotor::getTargetStep()   const { return targetStep_;   }
int32_t  StepperMotor::getLimitLeft()    const { return limitLeft_;    }
int32_t  StepperMotor::getLimitRight()   const { return limitRight_;   }
int8_t   StepperMotor::getStepPin()      const { return stepPin_;      }
int8_t   StepperMotor::getDirPin()       const { return dirPin_;       }
int8_t   StepperMotor::getEnablePin()    const { return enablePin_;    }
bool         StepperMotor::isRunning()       const { return motionState_ != MotionState::IDLE; }
bool         StepperMotor::isAtTarget()      const { return targetStep_ != NO_TARGET && motionState_ == MotionState::IDLE && currentStep_ == targetStep_; }
bool         StepperMotor::isAtLimit()       const { return hasLimits() && motionState_ == MotionState::IDLE && (currentStep_ == limitLeft_ || currentStep_ == limitRight_); }
float        StepperMotor::getAcceleration() const { return accel_;        }

float StepperMotor::getCurrentSpeed() const {
    if (motionState_ == MotionState::IDLE || cn_ == 0.0f) return 0.0f;
    return 1000000.0f / cn_ * (dirPositive_ ? 1.0f : -1.0f);
}

// ---- setAcceleration -------------------------------------------
StepperMotor& StepperMotor::setAcceleration(float a) {
    if (a <= 0.0f) return *this;
    if (accel_ > 0.0f && n_ > 0)
        n_ = (int32_t)((float)n_ * (accel_ / a));
    accel_ = a;
    c0_    = 0.676f * sqrtf(2.0f / accel_) * 1000000.0f;
    if (c0_ < (float)(MIN_STEP_PERIOD_US * 2)) c0_ = (float)(MIN_STEP_PERIOD_US * 2);
    return *this;
}

// ---- setLimits -------------------------------------------------
StepperMotor& StepperMotor::setLimits(int32_t left, int32_t right) {
    if (initialized_) return *this;
    if (left == right) return *this;
    limitLeft_  = left;
    limitRight_ = right;
    return *this;
}

// ---- Helpers geométricos ---------------------------------------

int32_t StepperMotor::stepsIncremental(int32_t from, int32_t to, uint32_t rev) {
    return (to - from + (int32_t)rev) % (int32_t)rev;
}

int32_t StepperMotor::shortestPath(int32_t from, int32_t to, uint32_t rev) {
    int32_t fwd = stepsIncremental(from, to, rev);
    int32_t bwd = stepsIncremental(to, from, rev);
    return (fwd <= bwd) ? fwd : -bwd;
}

bool StepperMotor::isInBounds(int32_t step) const {
    if (!hasLimits()) return true;
    int32_t toStep  = stepsIncremental(limitLeft_, step,        stepsPerRev_);
    int32_t toRight = stepsIncremental(limitLeft_, limitRight_, stepsPerRev_);
    return toStep <= toRight;
}

bool StepperMotor::hasLimits() const {
    return limitLeft_ != NO_TARGET && limitRight_ != NO_TARGET;
}

int32_t StepperMotor::distanceToLimit(int32_t limit) const {
    return stepsIncremental(currentStep_, limit, stepsPerRev_);
}

// ---- chooseDirection -------------------------------------------
// Con límites: compara distancia incremental current→target vs current→limitRight_.
//   Si el target está antes del límite derecho yendo incremental → incremental.
//   Si no → decremental (el target está del otro lado del arco).
// Sin límites: shortestPath dentro de tolerancia; si no, signo de targetSpeed_.
bool StepperMotor::chooseDirection() const {
    if (hasLimits() && targetStep_ != NO_TARGET) {
        int32_t toTarget = stepsIncremental(currentStep_, targetStep_,  stepsPerRev_);
        int32_t toRight  = stepsIncremental(currentStep_, limitRight_,  stepsPerRev_);
        return toTarget <= toRight;
    }

    if (targetStep_ != NO_TARGET) {
        int32_t dist = shortestPath(currentStep_, targetStep_, stepsPerRev_);
        if (labs(dist) <= (int32_t)tolerance_) return (dist > 0);
    }
    return (targetSpeed_ > 0.0f);
}


// ---- setTargetStep ---------------------------------------------
// Registra el objetivo de posición y calcula la dirección correcta.
// Con límites: solo acepta targets dentro del arco válido.
// Arranca desde IDLE si hay velocidad disponible.
StepperMotor& StepperMotor::setTargetStep(int32_t step) {
    if (step == NO_TARGET) {
        targetStep_       = NO_TARGET;
        brakingForTarget_ = false;
        return *this;
    }
    
    if (step < 0 || step >= (int32_t)stepsPerRev_) return *this;
    if (hasLimits() && !isInBounds(step))       return *this;

    targetStep_ = step;

    brakingForTarget_ = false;
    dirPositiveQueue_ = chooseDirection();

    startMoving();
    return *this;
}

// ---- setVelocity -----------------------------------------------
// Registra la velocidad objetivo y arranca desde IDLE si corresponde.
// No toca motionState_ si ya está en movimiento — update() se encarga.
StepperMotor& StepperMotor::setVelocity(float speed) {
    if (!initialized_) return *this;
    
    targetSpeed_    = speed;
    targetSpeedAbs_ = fabsf(speed);
    cMin_           = (targetSpeedAbs_ > 0.0f)
                      ? max(1000000.0f / targetSpeedAbs_, (float)(MIN_STEP_PERIOD_US * 2))
                      : c0_*2;

    dirPositiveQueue_ = chooseDirection();

    startMoving();
    return *this;
}


void StepperMotor::startMoving() {
    if (motionState_ != MotionState::IDLE) return;
    if (targetSpeedAbs_ <= 0.0f) return;
    auto dirLimit = dirPositiveQueue_ ? limitRight_ : limitLeft_;
    if(currentStep_ == dirLimit) return;
    if(targetStep_ == currentStep_) return;

    applyDirection();
    n_   = 0;
    cn_  = c0_;
    stepTimer_.setTimer((uint32_t)(cn_ / 2));
    stepTimer_.restart();
    motionState_ = MotionState::ACCEL;
}

// ---- update ----------------------------------------------------
void StepperMotor::update() {
    if (motionState_ == MotionState::IDLE) return;
    if (!stepTimer_.expired()) return;
    stepTimer_.accountPeriod();

    stepPinState_ = !stepPinState_;
    digitalWrite(stepPin_, stepPinState_ ? HIGH : LOW);

    // Lógica solo en flanco de subida — cuando el driver registra el paso.
    if (!stepPinState_) return;

    currentStep_ += (dirPositive_ ? 1 : -1);
    currentStep_  = (currentStep_ + (int32_t)stepsPerRev_) % (int32_t)stepsPerRev_;

    // ---- Lógica posicional: límites (prioridad máxima) ----------
    if (hasLimits()) {
        int32_t distToLimit = dirPositive_
            ? stepsIncremental(currentStep_, limitRight_, stepsPerRev_)
            : stepsIncremental(limitLeft_,   currentStep_, stepsPerRev_);

        if (distToLimit <= calcStepsToStop()) {
            if (!brakingForTarget_) {
                float v           = 1000000.0f / cn_;
                n_                = (int32_t)((v * v) / (2.0f * accel_));
                brakingForTarget_ = true;
                motionState_      = MotionState::DECEL;
            }

            // Force stop
            if(distToLimit == 0){
                brakingForTarget_ = false;
                motionState_ = MotionState::IDLE;
                stepTimer_.stop();
                return;
            }

            updateAcceleration();
            return;
        }
    }

    // ---- Lógica posicional: target ------------------------------
    if (targetStep_ != NO_TARGET) {
        int32_t dist = distanceToTarget();

        if (dist <= calcStepsToStop()) {
            if (!brakingForTarget_) {
                float v           = 1000000.0f / cn_;
                n_                = (int32_t)((v * v) / (2.0f * accel_));
                brakingForTarget_ = true;
                motionState_      = MotionState::DECEL;
            }
            updateAcceleration();
            return;
        }
    }

    // ---- Lógica de velocidad ------------------------------------
    if (!brakingForTarget_) {
        if (dirPositive_ != dirPositiveQueue_) {
            if (motionState_ != MotionState::DECEL) {
                float v      = 1000000.0f / cn_;
                n_           = (int32_t)((v * v) / (2.0f * accel_));
                motionState_ = MotionState::DECEL;
            }
        } else if (cMin_ < cn_) {
            if (motionState_ != MotionState::ACCEL) {
                float v      = 1000000.0f / cn_;
                n_           = (int32_t)((v * v) / (2.0f * accel_));
                motionState_ = MotionState::ACCEL;
            }
        } else if (cMin_ > cn_) {
            if (motionState_ != MotionState::DECEL) {
                float v      = 1000000.0f / cn_;
                n_           = (int32_t)((v * v) / (2.0f * accel_));
                motionState_ = MotionState::DECEL;
            }
        } else {
            if (motionState_ != MotionState::CRUISE)
                motionState_ = MotionState::CRUISE;
        }
    }

    updateAcceleration();
}

// ---- applyDirection --------------------------------------------
void StepperMotor::applyDirection() {
    dirPositive_ = dirPositiveQueue_;
    if (dirPin_ != -1)
        digitalWrite(dirPin_, (dirPositive_ != inverted_) ? HIGH : LOW);
}

// ---- updateAcceleration ----------------------------------------
void StepperMotor::updateAcceleration() {
    switch (motionState_) {
        case MotionState::ACCEL:
            n_++;
            cn_ -= (2.0f * cn_) / (4.0f * (float)n_ + 1.0f);
            if (cn_ <= cMin_) { cn_ = cMin_; motionState_ = MotionState::CRUISE; }
            break;

        case MotionState::CRUISE:
            cn_ = cMin_;
            break;

        case MotionState::DECEL:
            cn_ += (2.0f * cn_) / (4.0f * (float)n_ + 1.0f);
            n_--;
            if (n_ <= 0 || cn_ >= c0_) {
                cn_               = c0_;
                brakingForTarget_ = false;

                if (targetStep_ != NO_TARGET && currentStep_ == targetStep_) {
                    motionState_ = MotionState::IDLE;
                    stepTimer_.stop();
                    return;
                }

                if (hasLimits()) {
                    int32_t arrivedLimit = dirPositive_ ? limitRight_ : limitLeft_;
                    if (currentStep_ == arrivedLimit) {
                        motionState_ = MotionState::IDLE;
                        stepTimer_.stop();
                        return;
                    }
                }

                // Overshoot o target cambiado: recalcular dirección y rearrancar.
                dirPositiveQueue_ = chooseDirection();

                if (dirPositiveQueue_ != dirPositive_) {
                    applyDirection();
                }

                if (targetSpeedAbs_ > 0.0f) {
                    n_           = 0;
                    motionState_ = MotionState::ACCEL;
                } else {
                    motionState_ = MotionState::IDLE;
                    stepTimer_.stop();
                    return;
                }
            }
            break;

        case MotionState::IDLE:
        default:
            return;
    }
    stepTimer_.setTimer((uint32_t)(cn_ / 2));
}

// ---- calcStepsToStop -----------------------------------------
int32_t StepperMotor::calcStepsToStop() {
    // cn_ es el período completo en µs → v en steps/s = 1000000 / cn_
    float v      = 1000000.0f / cn_;
    return (int32_t)((v * v) / (2.0f * accel_));
}

// ---- distanceToTarget ------------------------------------------
int32_t StepperMotor::distanceToTarget() const {
    if (targetStep_ == NO_TARGET) return INT32_MAX;
    return dirPositive_
        ? stepsIncremental(currentStep_, targetStep_,  stepsPerRev_)
        : stepsIncremental(targetStep_,  currentStep_, stepsPerRev_);
}