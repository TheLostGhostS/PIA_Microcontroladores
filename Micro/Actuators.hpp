#pragma once
// ============================================================
//  Actuators.hpp
//  Clases de control físico: Updatable, Timer, StepperMotor.
//
//  StepperMotor reemplaza a UsageServo. Controla un stepper
//  bipolar con driver paso/dirección usando la rampa de
//  aceleración de Austin (1995) en punto flotante.
// ============================================================

#include <Arduino.h>
#include <stdint.h>     // compatible AVR/ESP/ARM — evita <cstdint>

// ============================================================
//  ServoWrapper — interfaz uniforme sobre las distintas
//  implementaciones de Servo por plataforma.
//
//  Teensy : PWMServo  — usa write(us) en lugar de writeMicroseconds
//  ESP32  : ESP32Servo — mismo API que Servo estándar
//  AVR/SAM/SAMD: Servo estándar de Arduino
//
//  Todo el código externo usa solo ServoWrapper, sin #ifdef.
// ============================================================
#if defined(ESP32) || defined(ESP8266)
  #include <ESP32Servo.h>
  using ServoBackend = Servo;
#else
  #include <Servo.h>
  using ServoBackend = Servo;
#endif

class ServoWrapper {
public:
    // Adjunta el pin. pin==-1 es legal (servo deshabilitado).
    void attach(int8_t pin) {
        pin_ = pin;
        if (pin_ == -1) return;
        servo_.attach(pin_);
    }

    // Escribe un valor en microsegundos [~1000, ~2000].
    // Normaliza la diferencia de API entre plataformas.
    void writeMicroseconds(int16_t us) {
        if (pin_ == -1) return;
        servo_.writeMicroseconds(us);
    }

    // Desadjunta el pin y libera el timer.
    void detach() {
        if (pin_ == -1) return;
        servo_.detach();
        pin_ = -1;
    }

    bool attached() { return servo_.attached(); }

private:
    ServoBackend servo_;
    int8_t       pin_ { -1 };
};
#include "Timer.hpp"
#include <limits.h>     // INT32_MAX, compatible AVR/ESP/ARM

// ============================================================
//  Constantes de motor
// ============================================================
constexpr uint32_t DEFAULT_STEPS_PER_REV { 40000 };
constexpr int          MIN_STEP_PERIOD_US    { 25    };  // período mínimo de semipaso, µs


// ============================================================
//  Updatable  — interfaz de polling para el loop principal
// ============================================================
class Updatable {
public:
    virtual void update() = 0;
    virtual bool begin()  = 0;
    virtual ~Updatable()  = default;
};


// ============================================================
//  StepperMotor
//
//  Uso típico:
//    StepperMotor motor(stepPin, dirPin, enablePin);
//    motor.begin();
//    motor.setAcceleration(20000.0f);   // steps/s²
//    motor.setVelocity(5000.0f);        // steps/s, con signo
//
//  Para posicionamiento:
//    motor.setTargetPosition(10000);    // steps absolutos [0, stepsPerRev)
//    motor.setVelocity(5000.0f);        // se respeta al llegar
//
//  Llamar motor.update() en cada iteración del loop.
// ============================================================
class StepperMotor : public Updatable {
public:
    static constexpr int32_t NO_TARGET = -1;

    explicit StepperMotor(int8_t stepPin, int8_t dirPin, int8_t enablePin, bool invert = false, 
                          uint32_t stepsPerRev = DEFAULT_STEPS_PER_REV);

    // Configura los pines. Debe llamarse en setup() antes de cualquier movimiento.
    // Solo puede llamarse una vez. Retorna false si los pines son inválidos o ya
    // fue inicializado.
    bool begin() override;

    // ---- Getters ------------------------------------------------
    uint32_t     getStepsPerRev()   const;
    int32_t      getCurrentStep()   const;
    int32_t      getTargetStep()    const;
    int32_t      getLimitLeft()     const;
    int32_t      getLimitRight()    const;
    int8_t       getStepPin()       const;
    int8_t       getDirPin()        const;
    int8_t       getEnablePin()     const;
    // Velocidad actual con signo (steps/s). 0 si está en IDLE.
    float        getCurrentSpeed()  const;
    bool         isRunning()        const;
    bool         isAtTarget()       const;
    bool         isAtLimit()        const;  // true si está detenido en un límite
    float        getAcceleration()  const;

    // ---- Setters ------------------------------------------------
    // Límites del rango válido. Solo tienen efecto antes de begin().
    // El rango válido es el arco que va de limitLeft a limitRight incrementando.
    // Si alguno es NO_TARGET (-1), no hay límites.
    StepperMotor& setLimits(int32_t limitLeft, int32_t limitRight);

    // Velocidad objetivo con signo. Positivo = dirección HIGH, negativo = LOW.
    // Si speed == 0 inicia frenado.
    // No mueve si begin() no fue llamado.
    StepperMotor& setVelocity(float speed);

    // Aceleración en steps/s². Reescala n para mantener consistencia en vuelo.
    StepperMotor& setAcceleration(float accelStepsPerSqSec);

    // Define el paso objetivo sin asignar velocidad.
    // El motor se moverá cuando setVelocity() sea llamado con la dirección correcta,
    // o inmediatamente si ya tiene velocidad en la dirección correcta.
    // Pasar NO_TARGET para cancelar el objetivo (modo velocidad pura).
    StepperMotor& setTargetStep(int32_t step);

    // ---- Loop ---------------------------------------------------
    void update() override;

    ~StepperMotor() = default;

private:
    // ---- Movimiento interno -------------------------------------
    void applyDirection();
    void updateAcceleration();
    void startMoving();
    int32_t calcStepsToStop();
    int32_t distanceToTarget()               const;
    int32_t distanceToLimit(int32_t limit)   const;
    bool isInBounds(int32_t step)            const;
    bool hasLimits()                         const;
    bool chooseDirection()                   const;

    static int32_t stepsIncremental(int32_t from, int32_t to, uint32_t rev);
    static int32_t shortestPath(int32_t from, int32_t to, uint32_t rev);

    // ---- Estado de pines ----------------------------------------
    int8_t       stepPin_;
    int8_t       dirPin_;
    int8_t       enablePin_;
    bool         initialized_ { false };
    bool         inverted_    { false };

    // ---- Posición, objetivo y límites ---------------------------
    uint32_t     stepsPerRev_;
    float        tolerance_   { 1.0f      };
    int32_t      currentStep_ { 0         };
    int32_t      targetStep_  { NO_TARGET };
    int32_t      limitLeft_   { NO_TARGET };
    int32_t      limitRight_  { NO_TARGET };

    // ---- Dirección ----------------------------------------------
    bool dirPositive_      { false };
    bool dirPositiveQueue_ { false };
    bool stepPinState_     { false };

    // ---- Máquina de estados de movimiento -----------------------
    enum class MotionState : uint8_t { IDLE, ACCEL, CRUISE, DECEL };
    MotionState motionState_ { MotionState::IDLE };

    bool brakingForTarget_ { false };  // frenado iniciado por lógica posicional

    // ---- Parámetros de velocidad / aceleración ------------------
    float targetSpeed_       { 0.0f     };
    float targetSpeedAbs_    { 0.0f     };
    float accel_             { 20000.0f };

    float    c0_    { 0.0f };
    float    cMin_  { 0.0f };
    float    cn_    { 0.0f };
    int32_t  n_     { 0    };

    Timer stepTimer_ {};
};


// ============================================================
//  DCMotor  — interfaz abstracta
//
//  Controla un motor DC vía ESC/controlador con señal PWM tipo servo.
//  La velocidad se expresa en [-100, 100] donde:
//    100  = máximo adelante  (maxPWM_)
//   -100  = máximo atrás     (minPWM_)
//      0  = parado           (centerPWM_)
// ============================================================
class DCMotor {
public:
    virtual void  setSpeed(float speed)              = 0;
    virtual void  setAcceleration(float accelPerSec) = 0;
    virtual float getSpeed()                   const = 0;
    virtual float getAcceleration()            const = 0;
    virtual bool  isMoving()                   const = 0;
    virtual void  stop()                             = 0;
    virtual ~DCMotor() = default;
};


// ============================================================
//  DCMotorCount<TEAM_SIZE>
//
//  Agrupa TEAM_SIZE pines PWM (servos) bajo una misma interfaz.
//  Todos los pines reciben el mismo valor PWM simultáneamente.
//
//  Uso típico:
//    DCMotorCount<2> motors(pin1, pin2);
//    motors.begin();
//    motors.setAcceleration(50.0f);
//    motors.setSpeed(75.0f);
//
//  Llamar motors.update() en cada iteración del loop.
// ============================================================
template <int TEAM_SIZE>
class DCMotorCount : public Updatable, public DCMotor {
public:
    static constexpr uint32_t UPDATE_PERIOD_US { 5000 };  // 5 ms entre pasos de aceleración

    template <typename... Args>
    explicit DCMotorCount(Args&&... args);

    bool begin() override;

    void  setSpeed(float speed)              override;
    void  stop()                             override;
    float getSpeed()                   const override;
    float getAcceleration()            const override;
    bool  isMoving()                   const override;
    void  setAcceleration(float accelPerSec) override;

    void update() override;

    ~DCMotorCount() = default;

private:
    void writePWM();

    int8_t       pwmPins_[TEAM_SIZE];
    ServoWrapper servos_[TEAM_SIZE];

    int16_t minPWM_    { 1000 };
    int16_t centerPWM_ { 1500 };
    int16_t maxPWM_    { 2000 };

    float currentSpeed_ { 0.0f  };
    float targetSpeed_  { 0.0f  };
    float acceleration_ { 50.0f };

    Timer updateTimer_ { UPDATE_PERIOD_US };
};


// ================================================================
//   DCMotorCount — implementaciones (template en header)
// ================================================================

template<int TEAM_SIZE>
template<typename... Args>
DCMotorCount<TEAM_SIZE>::DCMotorCount(Args&&... args)
    : pwmPins_ { static_cast<int8_t>(args)... }
{
    static_assert(TEAM_SIZE > 0, "Debe de haber minimo un elemento");
    static_assert(sizeof...(Args) == TEAM_SIZE,
        "El numero de pines debe coincidir con TEAM_SIZE");
}

template<int TEAM_SIZE>
bool DCMotorCount<TEAM_SIZE>::begin() {
    //Serial.println("Started a servo");
    for (uint8_t i = 0; i < TEAM_SIZE; i++) {
        if (pwmPins_[i] == -1) continue;
        //Serial.println("Pin: " + String(pwmPins_[i]));
        servos_[i].attach(pwmPins_[i]);
        servos_[i].writeMicroseconds(centerPWM_);
    }

    updateTimer_.restart();
    return true;
}

template<int TEAM_SIZE>
void DCMotorCount<TEAM_SIZE>::setSpeed(float speed) {
    //Serial.println("Setting speed to " + String(speed));
    targetSpeed_ = constrain(speed, -100.0f, 100.0f);
    if (acceleration_ == 0.0f) {
        currentSpeed_ = targetSpeed_;
        writePWM();
    }
}

template<int TEAM_SIZE>
void DCMotorCount<TEAM_SIZE>::stop() { setSpeed(0.0f); }

template<int TEAM_SIZE>
float DCMotorCount<TEAM_SIZE>::getSpeed()        const { return currentSpeed_; }
template<int TEAM_SIZE>
bool  DCMotorCount<TEAM_SIZE>::isMoving()        const { return currentSpeed_ != 0.0f; }
template<int TEAM_SIZE>
float DCMotorCount<TEAM_SIZE>::getAcceleration() const { return acceleration_; }

template<int TEAM_SIZE>
void DCMotorCount<TEAM_SIZE>::setAcceleration(float accelPerSec) {
    acceleration_ = accelPerSec;
}

template<int TEAM_SIZE>
void DCMotorCount<TEAM_SIZE>::writePWM() {
    int16_t pwm;
    if (currentSpeed_ >= 0.0f)
        pwm = centerPWM_ + (int16_t)((maxPWM_    - centerPWM_) *   currentSpeed_  / 100.0f);
    else
        pwm = centerPWM_ - (int16_t)((centerPWM_ - minPWM_)    * (-currentSpeed_) / 100.0f);

    for (uint8_t i = 0; i < TEAM_SIZE; i++) {
        if (pwmPins_[i] != -1) servos_[i].writeMicroseconds(pwm);
    }
}

template<int TEAM_SIZE>
void DCMotorCount<TEAM_SIZE>::update() {
    if (currentSpeed_ == targetSpeed_) return;
    if (!updateTimer_.expired())       return;
    updateTimer_.restart();

    constexpr float dt = DCMotorCount<TEAM_SIZE>::UPDATE_PERIOD_US / 1000000.0f;
    float delta = acceleration_ * dt;

    if (currentSpeed_ < targetSpeed_) {
        currentSpeed_ += delta;
        if (currentSpeed_ > targetSpeed_) currentSpeed_ = targetSpeed_;
    } else {
        currentSpeed_ -= delta;
        if (currentSpeed_ < targetSpeed_) currentSpeed_ = targetSpeed_;
    }

    writePWM();
}