#include <mbed.h>
#include "motor_lmtsw.hpp"
#include "motor_position.hpp"

class HinaDustpan
{
private:
    MotorLmtsw motor1;
    MotorPosition motor2;
    uint32_t md_update_timeout_us;
    uint32_t md_update_minimum_us;
    Timer md_update_timer;
    CAN* can;
    uint32_t can_id;
    int16_t prev_motor_value[2];
public:
    HinaDustpan(int16_t motor1_f_thrust, int16_t motor1_b_thrust, pid_param_t motor2_gain, uint32_t md_update_timeout_us, uint32_t md_update_minimum_us, CAN* can, uint32_t can_id)
    : motor1(motor1_f_thrust, motor1_b_thrust), motor2(motor2_gain)
    {
        this->md_update_timeout_us = md_update_timeout_us;
        this->md_update_minimum_us = md_update_minimum_us;
        this->can = can;
        this->can_id = can_id;
        md_update_timer.start();
    }
    void process(bool motor1_cmd, bool motor1_f_lmtsw, bool motor1_b_lmtsw, float motor2_cmd_target, float motor2_feedback){
        int16_t motor_value[2];
        if (motor1_cmd){
            motor1.forward();
        }else{
            motor1.backward();
        }
        motor_value[0] = motor1.process(motor1_f_lmtsw, motor1_b_lmtsw);
        motor_value[1] = motor2.process(motor2_feedback, motor2_cmd_target) * 0.95 * INT16_MAX ;
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
    }
};
