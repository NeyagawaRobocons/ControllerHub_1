#include <mbed.h>
#include "cylinder.hpp"
class DaizaClamp
{
private:
    Cylinder cylinder12;
    Cylinder cylinder3;
    Cylinder cylinder4;
    CAN* can;
    uint32_t can_id;
    uint8_t prev_state;
public:
    DaizaClamp(uint32_t cylinder12_delay_us, uint32_t cylinder3_delay_us, uint32_t cylinder4_delay_us, CAN* can, uint32_t can_id)
    : cylinder12(cylinder12_delay_us, cylinder12_delay_us), cylinder3(cylinder3_delay_us, cylinder3_delay_us), cylinder4(cylinder4_delay_us, cylinder4_delay_us)
    {
        this->can = can;
        this->can_id = can_id;
    }
    void process(bool cylinder12_cmd, bool cylinder3_cmd, bool cylinder4_cmd){
        if(cylinder12_cmd){
            cylinder12.forward();
        }else{
            cylinder12.backward();
        }
        if(cylinder3_cmd){
            cylinder3.forward();
        }else{
            cylinder3.backward();
        }
        if(cylinder4_cmd){
            cylinder4.forward();
        }else{
            cylinder4.backward();
        }
        CylinderValve cylinder12_valve = cylinder12.process();
        CylinderValve cylinder3_valve = cylinder3.process();
        CylinderValve cylinder4_valve = cylinder4.process();
        uint8_t state = 0;
        if(cylinder12_valve.forward_valve){
            state |= 0b00000001;
        }
        if(cylinder12_valve.backward_valve){
            state |= 0b00000010;
        }
        if(cylinder3_valve.forward_valve){
            state |= 0b00000100;
        }
        if(cylinder3_valve.backward_valve){
            state |= 0b00001000;
        }
        if(cylinder4_valve.forward_valve){
            state |= 0b00010000;
        }
        if(cylinder4_valve.backward_valve){
            state |= 0b00100000;
        }
        if(state != prev_state){
            CANMessage msg;
            msg.id = can_id;
            msg.len = 1;
            msg.data[0] = state;
            can->write(msg);
            prev_state = state;
        }
    }
    CylinderState get_cylinder12_state(){
        return cylinder12.get_state();
    }
    CylinderState get_cylinder3_state(){
        return cylinder3.get_state();
    }
    CylinderState get_cylinder4_state(){
        return cylinder4.get_state();
    }
};
