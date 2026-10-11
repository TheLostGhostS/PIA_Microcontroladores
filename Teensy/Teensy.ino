#include "Messaging.hpp"
#include "Timer.hpp"

constexpr int PinIN1 = 7;
constexpr int PinIN2 = 6;

template<int PinA, int PinB>
class Motor{

public:
    Motor(){
        pinMode(PinA, OUTPUT);
        pinMode(PinB, OUTPUT);
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
        if(in_speed > 100.0f || in_speed < -100.0f){
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

    Timer vel_count{0};
    float vel_percentage{0};
    bool running{false};
    // 50 milis
    uint32_t period{50000}; 
    
};

void setup() {
    Serial.begin(115200);
    while(!Serial);

    
}

void loop() {

}
