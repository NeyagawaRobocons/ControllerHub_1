#include <mbed.h>

#include "fifo.h"
#include "bfcobs.hpp"
#include "QEI_step.h"
#include "PID.h"

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
    float motor_target[3];
    
    while(1){
        if (serial.readable())
        {
            uint8_t data[32];
            ssize_t read_num = serial.read(data, 32);
            for (size_t i = 0; i < read_num; i++)
            {
                cobs.push(data[i]);
            }
            if(cobs.ready() > 1){
                size_t size;
                uint8_t buffer[64];
                int ret = cobs.read(buffer, &size);
                if(ret > 0){
                    // process messages
                    if(size == 13){
                        if (buffer[0] == 0x01)
                        {
                            for (size_t i = 0; i < 3; i++)
                            {
                                memcpy(&motor_target[i], &buffer[4*i+1], 4);
                            }
                        }
                    }
                }
            led = !led;
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
            if (button == 0)
            {
                motors[0] = INT16_MAX * 0.95 * 0.5;
                motors[1] = INT16_MAX * 0.95 * 0.5;
                motors[2] = INT16_MAX * 0.95 * 0.5;
                motors[3] = INT16_MAX * 0.95 * 0.5;
                write_motor(&can, 0x01, motors);
            }
            else
            {
                motors[0] = motor_target[0] / motor_gain * 0.95 * INT16_MAX;
                motors[1] = motor_target[1] / motor_gain * 0.95 * INT16_MAX;
                motors[2] = motor_target[2] / motor_gain * 0.95 * INT16_MAX;
                // motors[2] = (motor_target[2] / motor_gain + motor2_pid.process(motor2_speed, motor_target[2])) * 0.95 * INT16_MAX;
                motors[3] = 0;
                write_motor(&can, 0x01, motors);
            }
            
            motor_write_scheduler.reset();
        }
    }
}
