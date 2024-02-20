#include <mbed.h>
#include "PID.h"

class MotorPosition
{
private:
    PID pid;
public:
    MotorPosition(pid_param_t gain): pid(gain){
    }
    float process(float feedback, float target){
        return pid.process(feedback, target);
    }
    void set_gain(pid_param_t gain){
        pid.set_params(gain);
    }
    pid_param_t get_gain(){
        return pid.get_param();
    }
};