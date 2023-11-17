#include <mbed.h>

#include "fifo.h"
#include "COBS.h"
#include "PackedSerial.h"
#include "QEI_step.h"
#include "PID.h"

// #define debug


class data_array{
    public:
        data_array(int32_t count[3], int16_t speed[3]){
            for(int i = 0; i < 3; i++){
                this->count[i] = count[i];
            }
            for(int i = 0; i < 3; i++){
                this->speed[i] = speed[i];
            }
        }
        std::array<uint8_t, 19> pack(){
            std::array<uint8_t, 19> data;
            data[0] = 0x01;
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
        int32_t count[3];
        int16_t speed[3];
    private:
};

void write_motor(CAN can, unsigned int id, std::array<int16_t, 4> motors){
    CANMessage msg;
    msg.id = id;
    msg.len = 8;
    for(int i = 0; i < 4; i++){
        msg.data[2*i] = motors[i] >> 8;
        msg.data[2*i+1] = motors[i] & 0xff;
    }
    can.write(msg);
}

static DigitalOut led(LED1);

int main(){
    #ifndef debug
    BufferedSerial serial(CONSOLE_TX, CONSOLE_RX, 115200);
    struct data_type<1> target_format = {{0x01}, 12};
    struct data_type<1> pid_gain_format = {{0x02}, 12};
    std::array<struct data_type<1>, 2> rx_format = {target_format, pid_gain_format};
    PackedSerial<1, 2, 256> ps(&serial, rx_format);
    #endif
    Encoder encoder1(PC_4, PC_5, PullUp);
    Encoder encoder2(PC_6, PC_7, PullUp);
    Encoder encoder3(PC_8, PC_9, PullUp);
    CAN can(PA_11, PA_12, 1e6);

    std::array<PID, 3> motor_pid{PID{1,0,0},PID{1,0,0},PID{1,0,0}};

    Timer scheduler;
    scheduler.start();
    int wait_time = 20e3;
    
    while(1){
        led = !led;
        int32_t count[] = {encoder1, encoder2, encoder3};
        int16_t speed[] = {encoder1.get_speed(), encoder2.get_speed(), encoder3.get_speed()};
        data_array data2(count, speed);

        if(scheduler.elapsed_time().count() > wait_time){
            auto encoded_data = cobs_encode(data2.pack());
            #ifndef debug
            ps.send({0x01}, data2.pack());
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
    }
}
