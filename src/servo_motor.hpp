#include <mbed.h>

class Servo
{
private:
    PwmOut pwm;
    float duty_zero;
    float duty_per_rad;
public:
    Servo(PinName pin, float first_angle = 0, float period = 0.02, float duty_zero = 0.075, float duty_per_rad = 0.03023943918746)
    : pwm(pin) {
        pwm.period(period);
        pwm = duty_zero + first_angle * duty_per_rad;
        this->duty_zero = duty_zero;
        this->duty_per_rad = duty_per_rad;
    }
    void set(float rad){
        pwm = duty_zero + rad * duty_per_rad;
    }
};