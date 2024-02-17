#include <mbed.h>

#include "fifo.h"
#include "bfcobs.hpp"
#include "QEI_step.h"
#include "PID.h"

#include "mech.hpp"

// #define debug


class ododm_data_array{
    public:
        ododm_data_array(uint8_t header, float count[3], float speed[3]){
            this->header = header;
            for(int i = 0; i < 3; i++){
                this->count[i] = count[i];
            }
            for(int i = 0; i < 3; i++){
                this->speed[i] = speed[i];
            }
        }
        std::array<uint8_t, 25> pack(){
            std::array<uint8_t, 25> data;
            data[0] = header;
            for (size_t i = 0; i < 3; i++)
            {
                memcpy(&data[4*i+1], &count[i], 4);
            }
            for (size_t i = 0; i < 3; i++)
            {
                memcpy(&data[4*i+13], &speed[i], 4);
            }
            return data;
        } 
        uint8_t header;
        float count[3];
        float speed[3];
    private:
};

void write_motor(CAN* can, unsigned int id, std::array<int16_t, 4> motors){
    CANMessage msg;
    msg.id = id;
    msg.len = 8;
    for(int i = 0; i < 4; i++){
        msg.data[2*i] = (motors[i] >> 0) & 0xff;
        msg.data[2*i+1] = (motors[i] >> 8) & 0xff;
    }
    can->write(msg);
}

static DigitalOut led(LED1);

int main(){
    #ifndef debug
    BufferedSerial serial(CONSOLE_TX, CONSOLE_RX, 115200);
    bfcobs<64> cobs;
    #endif
    Encoder encoder1(PC_4, PC_5, 400, PullUp);
    Encoder encoder2(PC_6, PC_7, 400, PullUp);
    Encoder encoder3(PC_8, PC_9, 400, PullUp);
    Encoder encoder4(PC_10, PC_11, 400, PullUp);
    CAN can(PA_11, PA_12, 1e6);
    DigitalIn button(BUTTON1);

    Timer scheduler;
    scheduler.start();
    int wait_time = 10e3;

    Timer motor_write_scheduler;
    motor_write_scheduler.start();
    int motor_write_wait_time = 50e3;

    float motor_gain = 160.15962547712672;
    PID motor2_pid{0.1,0,0};
    float motor2_speed = encoder3.get_speed();
    float motor_target[4];

    pid_param_t hina_rot_gain{0.5, 0.05, 0};
    Mech mech(&can, 2, 20, 1000e3, 1000e3, 1000e3, 15000, -3000, hina_rot_gain,
    PA_1, PA_0, PullUp, true,
    PC_0, PA_15, PB_7, PC_1, PC_2, PC_3, PB_0, -4.4879895051283, 2.2439947525641);
    MechCmd cmd;
    Timer mech_state_serial_schduler;
    mech_state_serial_schduler.start();
    int mech_state_serial_wait_time = 100e3;
    
    while(1){
        if (serial.readable())
        {
            uint8_t data[32];
            ssize_t read_num = serial.read(data, 32);
            for (size_t i = 0; i < read_num; i++)
            {
                cobs.push(data[i]);
            }
            if(cobs.ready() > 3) led = 1; else led = 0;
            while(cobs.ready() > 1){
                size_t size;
                uint8_t buffer[64];
                int ret = cobs.read(buffer, &size);
                if(ret > 0){
                    // process messages
                    if(size == 17){
                        if (buffer[0] == 0x01)
                        {
                            //omni targets
                            for (size_t i = 0; i < 4; i++)
                            {
                                memcpy(&motor_target[i], &buffer[4*i+1], 4);
                            }
                        }
                    }
                    if(size == 2) {
                        if(buffer[0] == 0x02){
                            //daiza clamp
                            for (size_t i = 0; i < 4; i++)
                            {
                                cmd.daiza_cmd.cylinder[i] = (buffer[1] >> i) & 0x01;
                            }
                            // std::array<uint8_t, 6> debug_data;
                            // debug_data[0] = 0xff;
                            // for (size_t i = 0; i < 4; i++)
                            // {
                            //     debug_data[i+1] = (buffer[1] >> i) & 0x01;
                            // }
                            // debug_data[5] = cobs.ready();
                            // auto encoded_data = cobs_encode(debug_data);
                            // serial.write(encoded_data.data(), encoded_data.size());
                        }
                    }
                    if(size == 15) {
                        if(buffer[0] == 0x03){
                            //hina dustpan
                            for (size_t i = 0; i < 1; i++)
                            {
                                cmd.hina_cmd.motor_expand[i] = (buffer[1] >> i) & 0x01;
                            }for (size_t i = 0; i < 2; i++)
                            {
                                cmd.hina_cmd.cylinder[i] = (buffer[2] >> i) & 0x01;
                            }
                            for (size_t i = 0; i < 3; i++)
                            {
                                memcpy(&cmd.hina_cmd.motor_positions[i], &buffer[4*i+3], 4);
                            }
                            
                        }
                    }
                }
            // led = !led;
            }
        }
        

        if(scheduler.elapsed_time().count() > wait_time){
            float count[] = {encoder1, encoder2, encoder3};
            float speed[] = {encoder1.get_speed(), encoder2.get_speed(), encoder3.get_speed()};
            motor2_speed = speed[2];
            ododm_data_array odom_data(0x01, count, speed);
            auto encoded_data = cobs_encode(odom_data.pack());
            #ifndef debug
            serial.write(encoded_data.data(), encoded_data.size());
            #else
            printf("count: ");
            for(auto i : count){
                printf("%08x ", i);
            }
            printf("\n");
            printf("speed: ");
            for(auto i : speed){
                printf("%04x ", i);
            }
            printf("\n");
            // printf("encoded_data: ");
            // for(auto i : encoded_data){
            //     printf("%02x ", i);
            // }
            // printf("\n");
            #endif
            scheduler.reset();
        }

        if(motor_write_scheduler.elapsed_time().count() > motor_write_wait_time){
            std::array<int16_t, 4> motors;
            motors[0] = motor_target[0] / motor_gain * 0.95 * INT16_MAX;
            motors[1] = motor_target[1] / motor_gain * 0.95 * INT16_MAX;
            motors[2] = motor_target[2] / motor_gain * 0.95 * INT16_MAX;
            // motors[2] = (motor_target[2] / motor_gain + motor2_pid.process(motor2_speed, motor_target[2])) * 0.95 * INT16_MAX;
            motors[3] = motor_target[3] / motor_gain * 0.95 * INT16_MAX;
            write_motor(&can, 0x01, motors);
            
            motor_write_scheduler.reset();
        }

        MechProcessRet ret = mech.process(cmd);
        // led = cmd.daiza_cmd.cylinder[0];
        if(mech_state_serial_schduler.elapsed_time().count() > mech_state_serial_wait_time){
            std::array<uint8_t, 3> daiza_state;
            daiza_state[0] = 0x02;
            daiza_state[1] = 0;
            daiza_state[2] = 0;
            for (size_t i = 0; i < 1; i++)
            {
                daiza_state[1] |= ret.daiza_state.lmtsw[i] << i;
            }
            for (size_t i = 0; i < 4; i++)
            {
                daiza_state[2] |= ret.daiza_state.cylinder[i] << i;
            }
            auto encoded_data = cobs_encode(daiza_state);
            serial.write(encoded_data.data(), encoded_data.size());
            std::array<uint8_t, 6> hina_state;
            hina_state[0] = 0x03;
            hina_state[1] = 0;
            for (size_t i = 0; i < 5; i++)
            {
                hina_state[1] |= ret.hina_state.lmtsw[i] << i;
            }
            for (size_t i = 0; i < 1; i++)
            {
                memcpy(&hina_state[2+4*i], &ret.hina_state.potentiometer[i], 4);
            }
            auto encoded_data2 = cobs_encode(hina_state);
            serial.write(encoded_data2.data(), encoded_data2.size());
        }
    }
}
