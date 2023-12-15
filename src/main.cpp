#include <mbed.h>

#include "fifo.h"
#include "bfcobs.hpp"
#include "QEI_step.h"
#include "PID.h"

// #define debug


class ododm_data_array{
    public:
        ododm_data_array(uint8_t header, int32_t count[3], int16_t speed[3]){
            this->header = header;
            for(int i = 0; i < 3; i++){
                this->count[i] = count[i];
            }
            for(int i = 0; i < 3; i++){
                this->speed[i] = speed[i];
            }
        }
        std::array<uint8_t, 19> pack(){
            std::array<uint8_t, 19> data;
            data[0] = header;
            for (size_t i = 0; i < 3; i++)
            {
                //little endian
                data[4*i+1] = count[i] & 0xff;
                data[4*i+2] = (count[i] >> 8) & 0xff ;
                data[4*i+3] = (count[i] >> 16) & 0xff;
                data[4*i+4] = (count[i] >> 24) & 0xff;
            }
            for (size_t i = 0; i < 3; i++)
            {
                //little endian
                data[2*i+13] = speed[i] & 0xff;
                data[2*i+14] = (speed[i] >> 8) & 0xff;
            }
            return data;
        } 
        uint8_t header;
        int32_t count[3];
        int16_t speed[3];
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
    Encoder encoder1(PC_4, PC_5, PullUp);
    Encoder encoder2(PC_6, PC_7, PullUp);
    Encoder encoder3(PC_8, PC_9, PullUp);
    CAN can(PA_11, PA_12, 1e6);
    DigitalIn button(BUTTON1);

    std::array<PID, 3> motor_pid{PID{1,0,0},PID{1,0,0},PID{1,0,0}};

    Timer scheduler;
    scheduler.start();
    int wait_time = 10e3;

    Timer motor_write_scheduler;
    motor_write_scheduler.start();
    int motor_write_wait_time = 50e3;

    std::array<int16_t, 4> md1{0,0,0,0};
    
    while(1){
        if (serial.readable())
        {
            uint8_t data[32];
            ssize_t read_num = serial.read(data, 32);
            #ifndef debug
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
                    if(size == 7){
                        if (buffer[0] == 0x01)
                        {
                            md1[0] = (int16_t)((buffer[1] << 0) | (buffer[2] << 8));
                            md1[1] = (int16_t)((buffer[3] << 0) | (buffer[4] << 8));
                            md1[2] = (int16_t)((buffer[5] << 0) | (buffer[6] << 8));
                        }
                        
                    }
                }
            led = !led;
            }
            #else
            printf("%02x ", data);
            #endif
        }
        

        if(scheduler.elapsed_time().count() > wait_time){
            int32_t count[] = {encoder1, encoder2, encoder3};
            int16_t speed[] = {encoder1.get_speed(), encoder2.get_speed(), encoder3.get_speed()};
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
                write_motor(&can, 0x01, md1);
            }
            
            motor_write_scheduler.reset();
        }
    }
}
