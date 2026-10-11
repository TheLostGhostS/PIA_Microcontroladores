// ============================================================
//  teensy.ino  — aplicación principal
//  Incluir los tres módulos de la librería:
//    Messaging.hpp     → protocolo I2C, Writer/Reader, CRC, ISRs
//    Actuators.hpp     → StepperMotor, Updatable, Timer
//    TemplateTraits.hpp → MemberAction, ForEachAction, etc.
// ============================================================
#include "Messaging.hpp"
#include "Messaging_CAN.hpp"
#include "Actuators.hpp"
#include "TemplateTraits.hpp"
#include "Timer.hpp"

// ============================================================
//  Configuración de la aplicación
// ============================================================
constexpr int     NUM_STEP_MOTORS { 1    };
constexpr int     NUM_UPDATABLES  { 1    };

constexpr gpio_num_t TX_PIN = GPIO_NUM_21;
constexpr gpio_num_t RX_PIN = GPIO_NUM_20;

constexpr uint8_t  MY_ADDR   = 0x06;
constexpr uint8_t  HOST_ADDR = 0 - MY_ADDR;
constexpr uint16_t CHANNEL   = 0;

// ============================================================
//  Motores
// ============================================================
StepperMotor stepperMotor {  2, 3, 4, false, 5000 };


StepperMotor*  stepperMotorArray [NUM_STEP_MOTORS]  { &stepperMotor };
Updatable*     updatables        [NUM_UPDATABLES]   { &stepperMotor };

// ============================================================
//  Definición de globales del protocolo (extern en Messaging.hpp)
// ============================================================
MsgHandler handlers[HANDLERS]     = { nullptr };
MsgHandler writes[WRITE_HANDLERS] = { nullptr };
MsgHandler reads[READ_HANDLERS]   = { nullptr };

TxProtocol tx;
RxProtocol rx;

template<typename Action, bool Masked = false>
using ForEachStepper = ForEachAction<stepperMotorArray, NUM_STEP_MOTORS, Action, Masked>;

// ============================================================
//  Deadman
// ============================================================
void deadmanStop(){
    for(int i = 0; i < NUM_STEP_MOTORS; i ++){
        stepperMotorArray[i]->setVelocity(0.0f);
    }
}

// 100 ms
Deadman deadman{ 1000000, &deadmanStop };

// ============================================================
//  setup
// ============================================================
void setup() {
    Serial.begin(115200);

    RxSetResponder(rx, tx);

    const float accel = 500.0f;

    for(int i = 0; i < NUM_UPDATABLES; i++)
        updatables[i]->begin();

    for(int i = 0; i < NUM_STEP_MOTORS; i++){
        stepperMotorArray[i]->setAcceleration(accel).setTargetStep(StepperMotor::NO_TARGET);
    }

    // Configurar TWAI (mismo estilo que tus archivos de ejemplo)
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Filtro de hardware: solo aceptar frames con rx_id (29-bit extended)
    uint32_t rx_id = can_make_id(HOST_ADDR, MY_ADDR, CHANNEL);
    twai_filter_config_t f_config = {
        .acceptance_code = rx_id << 3,   // TWAI filtra por los 29 bits altos
        .acceptance_mask = 0x00000007,   // los 3 bits bajos son flags, no filtrar
        .single_filter   = true
    };
    //Serial.println("My id is: " + String(rx_id));

    /*/
    f_config = {
        .acceptance_code = 0x00000000,
        .acceptance_mask = 0xFFFFFFFF,
        .single_filter   = true
    };
    //*/

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    // Configurar IDs en TxProtocol
    uint32_t tx_id = can_make_id(MY_ADDR, HOST_ADDR, CHANNEL);
    tx.setTxId(tx_id);
    setRxId(rx_id);

    // Registrar handlers de mensajes
    // handlers[1] = ping;  // descomenta si quieres ping
    // handlers[2] = myWriteHandler;
    // handlers[3] = myReadHandler;

    // Lanzar tarea de recepción CAN en core 0
    // Loop() corre en core 1 (default de Arduino/ESP32)
    xTaskCreatePinnedToCore(
        canRxTask,   // función
        "canRx",     // nombre
        2048,        // stack bytes
        nullptr,     // param
        10,          // prioridad (alta para no perder frames)
        nullptr,     // handle (no necesario)
        0            // core 0
    );

    handlers[1] = ping;
    handlers[2] = handlerCaller<writes, WRITE_HANDLERS>;
    handlers[3] = handlerCaller<reads,  READ_HANDLERS>;

    // Escrituras
    registerHandler< 
        ActionSequence<
            InstanceAction<&rx, MemberAction<&RxProtocol::resetByteLoss>>>
    >(writes, 0);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::setVelocity>>
    >(writes, 1);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::setTargetStep>>
    >(writes, 2);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::setAcceleration>>
    >(writes, 3);

    // Lecturas
    registerHandler< 
        ActionSequence<
            InstanceAction<&rx, MemberAction<&RxProtocol::getByteLoss>>> 
        >(reads, 0);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::getCurrentSpeed>>
        >(reads, 1);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::getCurrentStep>>
        >(reads, 2);

    registerHandler<
        ForEachStepper<MemberAction<&StepperMotor::getAcceleration>>
        >(reads, 3);

}

// ============================================================
//  loop
// ============================================================

void loop() {
    
    for (int i = 0; i < NUM_UPDATABLES; i++)
        updatables[i]->update();

    rx.processPackets();

    deadman.check();
}
