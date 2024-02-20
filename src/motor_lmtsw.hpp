#include <mbed.h>
class MotorLmtsw
{
private:
    int16_t forward_thrust;
    int16_t backward_thrust;
    bool is_forwarding;
    bool is_backwarding;
public:
    MotorLmtsw(int16_t forward_thrust, int16_t backward_thrust){
        this->forward_thrust = forward_thrust;
        this->backward_thrust = backward_thrust;
    }
    int16_t process(bool forward_lmtsw, bool backward_lmtsw){
        if(is_forwarding){
            if(forward_lmtsw){
                is_forwarding = false;
                return 0;
            }else{
                return forward_thrust;
            }
        }
        if(is_backwarding){
            if(backward_lmtsw){
                is_backwarding = false;
                return 0;
            }else{
                return backward_thrust;
            }
        }
        return 0;
    }
    void forward(){
        is_backwarding = false;
        is_forwarding = true;
    }
    void backward(){
        is_forwarding = false;
        is_backwarding = true;
    }
    void stop(){
        is_forwarding = false;
        is_backwarding = false;
    }
    void set_forward_thrust(int16_t forward_thrust){
        this->forward_thrust = forward_thrust;
    }
    void set_backward_thrust(int16_t backward_thrust){
        this->backward_thrust = backward_thrust;
    }
    int16_t get_forward_thrust(){
        return forward_thrust;
    }
    int16_t get_backward_thrust(){
        return backward_thrust;
    }
};