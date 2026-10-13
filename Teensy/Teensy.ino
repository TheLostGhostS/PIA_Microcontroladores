#include "Timer.hpp"

constexpr int PinIN1 = 7;
constexpr int PinIN2 = 6;

class Motor{
public:

    Motor() = default;
    Motor(int pinA, int pinB){
        initialize(pinA, pinB);
    }

    void initialize(int pinA, int pinB){
        if(PinA < 0 && pinA >= 0){
            PinA = pinA;
            pinMode(PinA, OUTPUT);
            digitalWrite(PinA, LOW);
        }

        if(PinB < 0 && pinB >= 0){
            PinB = pinB;
            pinMode(PinB, OUTPUT);
            digitalWrite(PinB, LOW);
        }
    }

    void update(){
        if(!vel_count.active() || !vel_count.expired()){
            return;
        }

        int work_pin = vel_percentage < 0 ? PinA : PinB;
        float abs_vel = abs(vel_percentage)/100;
        uint32_t cooldown {period * (running ? (1-abs_vel) : abs_vel)};
        vel_count.setTimer(cooldown);

        if(!vel_count.active()){
            return;
        }

        running = !running;
        digitalWrite(work_pin, running);
        vel_count.restart();

    };

    void changeSpeed(float in_speed){
        if(PinA < 0 || PinB < 0){
            return;
        }

        if(in_speed > 100.0f || in_speed < -100.0f){
            Serial.println(in_speed);
            return;
        }

        vel_percentage = in_speed;
        digitalWrite(PinA, LOW);
        digitalWrite(PinB, LOW);
        running = false;

        if(vel_percentage == 0){
            vel_count.setTimer(0);
        }else{
            vel_count.setTimer(1);
            vel_count.restart();
        }
    }

    void updateInterval(uint32_t milis_, bool is_micros = false){
        period = milis_;
        if(is_micros)
            period *= 1000;
    }
 
    void forward()  { changeSpeed(  100 );  }
    void backward() { changeSpeed( -100 );  }
    void stop()     { changeSpeed(  0   );  }
    
private:

    int PinA{-1}, PinB{-1};
    Timer vel_count{0};
    float vel_percentage{0};
    bool running{false};
    // 50 milis
    uint32_t period{50000}; 
    
};


Timer ticking{1000000};
int state{1};
Motor DC1;
void setup() {
    Serial.begin(115200);
    while(!Serial);

    DC1.initialize(PinIN1, PinIN2);
  	ticking.restart();
}


void loop() {
    DC1.update();
    if(ticking.expired()){
        ticking.restart();
        switch(state){
            case 1:
            DC1.changeSpeed(50);
            break;
            case 2:
            DC1.changeSpeed(-50);
            break;
            default:
            state = 0;
            DC1.stop();
        }
        state++;
    }
    
}
