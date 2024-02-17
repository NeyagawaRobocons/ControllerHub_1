#include <mbed.h>
#include "motor_lmtsw.hpp"
#include "motor_position.hpp"
#include "servo_motor.hpp"

class HinaDustpan
{
private:
    MotorLmtsw motor1;
    MotorPosition motor2;
    Servo servo1;
    Servo servo2;
    uint32_t md_update_timeout_us;
    uint32_t md_update_minimum_us;
    Timer md_update_timer;
    Timer sol_latch1_timer;
    Timer sol_latch2_timer;
    bool sol_latch1_triggered;
    bool sol_latch2_triggered;
    CAN* can;
    uint32_t can_id;
    int16_t prev_motor_value[2];
public:
    HinaDustpan(int16_t motor1_f_thrust, int16_t motor1_b_thrust, pid_param_t motor2_gain, PinName pin_servo1, PinName pin_servo2, uint32_t md_update_timeout_us, uint32_t md_update_minimum_us, CAN* can, uint32_t can_id)
    : motor1(motor1_f_thrust, motor1_b_thrust), motor2(motor2_gain), servo1(pin_servo1), servo2(pin_servo2)
    {
        this->md_update_timeout_us = md_update_timeout_us;
        this->md_update_minimum_us = md_update_minimum_us;
        this->can = can;
        this->can_id = can_id;
        md_update_timer.start();
        sol_latch1_triggered = false;
        sol_latch2_triggered = false;
        sol_latch1_timer.start();
        sol_latch2_timer.start();
    }
    //return 2 solenoids state
    uint8_t process(bool motor1_cmd, bool motor1_f_lmtsw, bool motor1_b_lmtsw, float motor2_cmd_target, float motor2_feedback, float servo1_cmd, float servo2_cmd, bool sol_latch1_trig, bool sol_latch2_trig){
        int16_t motor_value[2];
        if (motor1_cmd){
            motor1.forward();
        }else{
            motor1.backward();
        }
        motor_value[0] = motor1.process(motor1_f_lmtsw, motor1_b_lmtsw);
        motor_value[1] = motor2.process(motor2_feedback, motor2_cmd_target) * 0.95 * INT16_MAX ;
        servo1.set(servo1_cmd);
        servo2.set(servo2_cmd);
        if(md_update_timer.elapsed_time().count() > md_update_minimum_us){
            CANMessage msg;
            msg.id = can_id;
            msg.len = 8;
            for (size_t i = 0; i < 2; i++)
            {
                msg.data[i*2] = motor_value[i] & 0xff;
                msg.data[1 + i*2] = (motor_value[i] >> 8) & 0xff;
            }
            can->write(msg);
            for (size_t i = 0; i < 2; i++)
            {
                prev_motor_value[i] = motor_value[i];
            }
            md_update_timer.reset();
        }
        if(sol_latch1_trig && !sol_latch1_triggered){
            sol_latch1_timer.reset();
            sol_latch1_triggered = true;
        }
        if(sol_latch2_trig && !sol_latch2_triggered){
            sol_latch2_timer.reset();
            sol_latch2_triggered = true;
        }
        if(sol_latch1_timer.elapsed_time().count() > 500e3){
            sol_latch1_triggered = false;
        }
        if(sol_latch2_timer.elapsed_time().count() > 500e3){
            sol_latch2_triggered = false;
        }
        uint8_t sol_state = 0;
        if(sol_latch1_triggered){
            sol_state |= 0b00000001;
        }
        if(sol_latch2_triggered){
            sol_state |= 0b00000010;
        }
        return sol_state;
    }
};
