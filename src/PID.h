#pragma once
#include "mbed.h"

struct pid_param_t
{
    float k_p;
    float k_i;
    float k_d;
};


class PID
{
private:
    float integral;
    float error_prev;
    int64_t t_prev_us;
    Timer timer;

    pid_param_t gain;
    float integral_max;
public:
    PID(float k_i, float k_p, float k_d, float integral_max, float first_feedback, float first_target);
    PID(pid_param_t gain, float first_feedback, float first_target);
    ~PID();
    float process(float feedback, float target);
    void reset();
    pid_param_t get_param();
    void set_params(pid_param_t gain);
};

PID::PID(float k_i, float k_p, float k_d, float integral_max = 1, float first_feedback = 0, float first_target = 0) : timer()
{
    this->gain.k_i = k_i;
    this->gain.k_p = k_p;
    this->gain.k_d = k_d;
    this->integral_max = integral_max;
    this->timer.start();
    this->t_prev_us = timer.elapsed_time().count();
    this->error_prev = first_target - first_feedback;
}

PID::PID(pid_param_t gain, float first_feedback, float first_target) : timer()
{
    this->gain = gain;
    this->timer.start();
    this->t_prev_us = timer.elapsed_time().count();
    this->error_prev = first_target - first_feedback;
}

float PID::process(float feedback, float target){
    int64_t t_us = timer.elapsed_time().count();
    float delta = (float)(t_us - t_prev_us) * 1.0E-6f;
    float error = target - feedback;
    this->integral += error * delta;
    if(this->integral > this->integral_max) this->integral = this->integral_max;
    if(this->integral < -this->integral_max) this->integral = -this->integral_max;    
    float difference = (error - error_prev) / delta;
    this->t_prev_us = t_us;
    return this->gain.k_p * error + this->gain.k_i * this->integral + this->gain.k_d * difference;
}

void PID::reset(){
    this->integral = 0;
    this->timer.reset();
    this->t_prev_us = timer.elapsed_time().count();
    this->error_prev = 0;
}

pid_param_t PID::get_param(){
    return this->gain;
}

void PID::set_params(pid_param_t gain){
    this->gain = gain;
}
