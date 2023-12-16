#pragma once
#include <mbed.h>

class Encoder {
public:
    Encoder(PinName pin_a, PinName pin_b, uint16_t resolution, PinMode pinmode = PullNone) : encoder_a(pin_a, pinmode), encoder_b(pin_b, pinmode), timer(){
        encoder_a.rise(callback(this, &Encoder::rise_a));
        encoder_b.rise(callback(this, &Encoder::rise_b));
        encoder_a.fall(callback(this, &Encoder::fall_a));
        encoder_b.fall(callback(this, &Encoder::fall_b));
        this->resolution4 = resolution;
        count = 0;
        prev_count = 0;
        timer.start();
        prev_t_us = timer.elapsed_time().count();
    }
    void reset(){
        count = 0;
    }
    operator float(){
        return (float)count / resolution4 * 2 * M_PI;
    }
    //return speed in radian per second  [rad/sec]
    float get_speed(){
        int64_t t_us = timer.elapsed_time().count();
        float delta = (float)(t_us - prev_t_us) * 1.0E-6f;
        float speed = (float)((count - prev_count) * 1)  / resolution4 * 2 * M_PI / delta ;
        prev_t_us = t_us;
        prev_count = count;
        return speed;
    }
private:
    void rise_a(){
        if(encoder_b.read()){
            count++;
        }else{
            count--;
        }
    }
    void rise_b(){
        if(encoder_a.read()){
            count--;
        }else{
            count++;
        }
    }
    void fall_a(){
        if(encoder_b.read()){
            count--;
        }else{
            count++;
        }
    }
    void fall_b(){
        if(encoder_a.read()){
            count++;
        }else{
            count--;
        }
    }
    InterruptIn encoder_a;
    InterruptIn encoder_b;
    Timer timer;
    uint16_t resolution4;
    int64_t prev_t_us;
    int32_t prev_count;
    int32_t count;
};
