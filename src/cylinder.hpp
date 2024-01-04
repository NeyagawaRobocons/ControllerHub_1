#include <mbed.h>

struct CylinderValve{
    bool forward_valve;
    bool backward_valve;
};

enum class CylinderState{
    FORWARDED,
    BACKWARDED,
    FORWARDING,
    BACKWARDING,
    STOPPED_UNKNOWN_POSITION,
};

class Cylinder
{
private:
    uint32_t forward_time_us;
    uint32_t backward_time_us;
    Timer timer;
    bool is_forwarding;
    bool is_backwarding;
    bool is_forwarded;
    bool is_backwarded;
public:
    Cylinder(uint32_t forward_time_us, uint32_t backward_time_us){
        this->forward_time_us = forward_time_us;
        this->backward_time_us = backward_time_us;
        timer.start();
    }
    CylinderValve process(){
        if(is_forwarding){
            is_backwarded = false;
            if(timer.elapsed_time().count() > forward_time_us){
                is_forwarding = false;
                is_forwarded = true;
                timer.reset();
            }
        }
        if(is_backwarding){
            is_forwarded = false;
            if(timer.elapsed_time().count() > backward_time_us){
                is_backwarding = false;
                is_backwarded = true;
                timer.reset();
            }
        }
        return {is_forwarding, is_backwarding};
    }
    void forward(){
        if(is_forwarding || is_forwarded){
            return;
        }
        is_backwarding = false;
        is_forwarding = true;
        timer.reset();
    }
    void backward(){
        if(is_backwarding || is_backwarded){
            return;
        }
        is_forwarding = false;
        is_backwarding = true;
        timer.reset();
    }
    void stop(){
        is_forwarding = false;
        is_backwarding = false;
        timer.reset();
    }
    CylinderState get_state(){
        if(is_forwarding){
            return CylinderState::FORWARDING;
        }
        if(is_backwarding){
            return CylinderState::BACKWARDING;
        }
        if(is_forwarded){
            return CylinderState::FORWARDED;
        }
        if(is_backwarded){
            return CylinderState::BACKWARDED;
        }
        return CylinderState::STOPPED_UNKNOWN_POSITION;
    }
    void set_forward_time_us(uint32_t forward_time_us){
        this->forward_time_us = forward_time_us;
    }
    void set_backward_time_us(uint32_t backward_time_us){
        this->backward_time_us = backward_time_us;
    }
    uint32_t get_forward_time_us(){
        return forward_time_us;
    }
    uint32_t get_backward_time_us(){
        return backward_time_us;
    }
};
