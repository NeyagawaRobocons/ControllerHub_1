#include <mbed.h>

#include <daiza_clamp.hpp>
#include <hina_dustpan.hpp>

template<size_t N_motor_positions, size_t N_motor_expand, size_t N_cylinder>
struct ActuatorCmd{
    float motor_positions[N_motor_positions];
    bool motor_expand[N_motor_expand];
    bool cylinder[N_cylinder];
};

struct MechCmd{
    ActuatorCmd<0,0,4> daiza_cmd;
    ActuatorCmd<1,1,0> hina_cmd;
};

template<size_t N_lmtsw, size_t N_cylinder, size_t N_potentiometer>
struct SensorState{
    bool lmtsw[N_lmtsw];
    bool cylinder[N_cylinder];
    float potentiometer[N_potentiometer];
};

class MechProcessRet{
public:
    SensorState<1,4,0> daiza_state;
    SensorState<5,0,1> hina_state;
    bool operator == (const MechProcessRet& other){
        bool ret = true;
        for (size_t i = 0; i < 1; i++)
        {
            ret &= daiza_state.lmtsw[i] == other.daiza_state.lmtsw[i];
        }
        for (size_t i = 0; i < 4; i++)
        {
            ret &= daiza_state.cylinder[i] == other.daiza_state.cylinder[i];
        }
        for (size_t i = 0; i < 5; i++)
        {
            ret &= hina_state.lmtsw[i] == other.hina_state.lmtsw[i];
        }
        for (size_t i = 0; i < 1; i++)
        {
            ret &= hina_state.potentiometer[i] == other.hina_state.potentiometer[i];
        }
        return ret;
    }
    bool operator != (const MechProcessRet& other){
        return !(*this == other);
    }
};

class Mech
{
private:
    DaizaClamp daiza;
    HinaDustpan hina;
    bool pin_invert;
    DigitalIn daiza_lmtsw;
    DigitalIn hina_up_lmtsw;
    DigitalIn hina_down_lmtsw;
    DigitalIn wall_1_lmtsw;
    DigitalIn wall_2_lmtsw;
    DigitalIn hina_rot_reset_lmtsw;
    AnalogIn hina_rot_angle_meter;
    float hina_rot_volt_to_rad_gain;
public:
    Mech(CAN* can, uint32_t can_id_md, uint32_t can_id_solenoid,
        uint32_t daiza_clamp_delay_us, uint32_t daiza_asm_delay_us, uint32_t daiza_up_delay_us,
        int16_t hina_up_thrust, int16_t hina_down_thrust, pid_param_t hina_rot_gain,
        PinMode lmtsw_pinmode, bool pin_invert, PinName daiza, PinName hina_up, PinName hina_down,
        PinName wall_1, PinName wall_2, PinName hina_rot_reset, PinName hina_rot_angle,
        float hina_rot_volt_to_rad_gain)
    : daiza(daiza_clamp_delay_us, daiza_asm_delay_us, daiza_up_delay_us, can, can_id_solenoid),
        hina(hina_up_thrust, hina_down_thrust, hina_rot_gain, 100e3, 50e3, can, can_id_md),
        daiza_lmtsw(daiza, lmtsw_pinmode), hina_up_lmtsw(hina_up, lmtsw_pinmode), hina_down_lmtsw(hina_down, lmtsw_pinmode),
        wall_1_lmtsw(wall_1, lmtsw_pinmode), wall_2_lmtsw(wall_2, lmtsw_pinmode), hina_rot_reset_lmtsw(hina_rot_reset, lmtsw_pinmode),
        hina_rot_angle_meter(hina_rot_angle)
    {
        this->hina_rot_volt_to_rad_gain = hina_rot_volt_to_rad_gain;
        this->pin_invert = pin_invert;
    }
    MechProcessRet process(MechCmd cmd){
        float dustpan_angle = hina_rot_angle_meter.read()*hina_rot_volt_to_rad_gain;
        daiza.process(cmd.daiza_cmd.cylinder[0], cmd.daiza_cmd.cylinder[2], cmd.daiza_cmd.cylinder[3]);
        hina.process(cmd.hina_cmd.motor_expand[0], hina_up_lmtsw * pin_invert, hina_down_lmtsw * pin_invert, cmd.hina_cmd.motor_positions[0], dustpan_angle);
        MechProcessRet ret;
        ret.daiza_state.lmtsw[0] = pin_invert ? !daiza_lmtsw : daiza_lmtsw;
        ret.daiza_state.cylinder[0] = daiza.get_cylinder12_state() == CylinderState::FORWARDED ||daiza.get_cylinder12_state() == CylinderState::BACKWARDING;
        ret.daiza_state.cylinder[1] = daiza.get_cylinder12_state() == CylinderState::FORWARDED ||daiza.get_cylinder12_state() == CylinderState::BACKWARDING;
        ret.daiza_state.cylinder[2] = daiza.get_cylinder3_state() == CylinderState::FORWARDED ||daiza.get_cylinder3_state() == CylinderState::BACKWARDING;
        ret.daiza_state.cylinder[3] = daiza.get_cylinder4_state() == CylinderState::FORWARDED ||daiza.get_cylinder4_state() == CylinderState::BACKWARDING;
        ret.hina_state.lmtsw[0] = pin_invert ? !hina_up_lmtsw : hina_up_lmtsw;
        ret.hina_state.lmtsw[1] = pin_invert ? !hina_down_lmtsw : hina_down_lmtsw;
        ret.hina_state.lmtsw[2] = pin_invert ? !wall_1_lmtsw : wall_1_lmtsw;
        ret.hina_state.lmtsw[3] = pin_invert ? !wall_2_lmtsw : wall_2_lmtsw;
        ret.hina_state.lmtsw[4] = pin_invert ? !hina_rot_reset_lmtsw : hina_rot_reset_lmtsw;
        ret.hina_state.potentiometer[0] = dustpan_angle;
        return ret;
    }
};