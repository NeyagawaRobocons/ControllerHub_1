#pragma once
#include <mbed.h>

class Encoder {
public:
    Encoder(PinName pin_a, PinName pin_b, us_timestamp_t calc_period, PinMode pinmode = PullNone) : encoder_a(pin_a, pinmode), encoder_b(pin_b, pinmode), ticker(){
        encoder_a.rise(callback(this, &Encoder::rise_a));
        encoder_b.rise(callback(this, &Encoder::rise_b));
        encoder_a.fall(callback(this, &Encoder::fall_a));
        encoder_b.fall(callback(this, &Encoder::fall_b));
        this->ticker.attach_us(callback(this, &Encoder::calculate_speed), calc_period);
        count = 0;
        speed = 0;
        this->calc_period = calc_period;
    }
    void reset(){
        count = 0;
    }
    operator int32_t(){
        return count;
    }
    //return speed in count per second with 4bit fraction
    int16_t get_speed(){
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
    void calculate_speed(){
        speed = (count - prev_count) * 16 * (1000000 / calc_period); // 16 is 4bit fraction
        prev_count = count;
    }
    InterruptIn encoder_a;
    InterruptIn encoder_b;
    Ticker ticker;
    us_timestamp_t calc_period;
    int32_t prev_count;
    int32_t count;
    int16_t speed;
};
