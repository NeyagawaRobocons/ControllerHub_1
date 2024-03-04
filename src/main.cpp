#include <main.h>
#include <mbed.h>
#include "stm32f4xx_hal.h"

#include "fifo.h"
#include "bfcobs2.hpp"
#include "QEI_step.h"
#include "PID.h"

#include "mech.hpp"

// #define debug
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

static void MX_TIM3_Init(void);


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
    bfcobs<128> cobs;
    #endif
    // Encoder encoder[4] = {
    //     Encoder(PC_4, PC_5, 400),
    //     Encoder{PC_6, PC_7, 400},
    //     Encoder{PC_8, PC_9, 400},
    //     Encoder{PC_10, PC_11, 400},
    // };
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

    float motor_gain = 160.15962547712672 * 2;
    PID motor2_pid{0.1,0,0};
    PID motor_pid[4] = {
        PID(0.1, 0.0, 0, 0.5),
        PID(0.1, 0.0, 0, 0.5),
        PID(0.1, 0.0, 0, 0.5),
        PID(0.1, 0.0, 0, 0.5),
    };
    float motor_speeds[4];
    motor_speeds[0] = -encoder1.get_speed();
    motor_speeds[1] = -encoder2.get_speed();
    motor_speeds[2] = -encoder3.get_speed();
    motor_speeds[3] = -encoder4.get_speed();
    float motor_target[4];

    pid_param_t hina_rot_gain{0.3, 0.05, 0};
    Mech mech(&can, 2, 20, 1000e3, 1000e3, 1000e3, 15000, -6000, hina_rot_gain,
    PA_1, PA_0, PullUp, true,
    PC_0, PA_15, PB_7, PC_1, PC_2, PC_3, PB_0, -3.5903916041026, 1.7951958020513);
    MechCmd cmd;
    cmd.hina_cmd.motor_positions[0] = - 3.14159265358979323846f / 2.0f;
    Timer mech_state_serial_schduler;
    mech_state_serial_schduler.start();
    int mech_state_serial_wait_time = 100e3;

    bool prev_bonbori_state = false;
    MX_TIM3_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 15);
    wait_us(1.2e3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    
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
                    if(size == 17) if (buffer[0] == 0x01){
                        //omni targets
                        for (size_t i = 0; i < 4; i++)
                        {
                            memcpy(&motor_target[i], &buffer[4*i+1], 4);
                        }
                    }
                    if(size == 2) if(buffer[0] == 0x02){
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
                    if(size == 15) if(buffer[0] == 0x03){
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
                    if(size == 2) if(buffer[0] == 0x04){
                        // bonbori
                        if(buffer[1] && !prev_bonbori_state){
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 29);
                            wait_us(1.2e3);
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        }else if(!buffer[1] && prev_bonbori_state){
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 15);
                            wait_us(1.2e3);
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        }else{
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        }
                        prev_bonbori_state = buffer[1];
                    }
                }
            // led = !led;
            }
        }
        

        if(scheduler.elapsed_time().count() > wait_time){
            motor_speeds[0] = -encoder1.get_speed();
            motor_speeds[1] = -encoder2.get_speed();
            motor_speeds[2] = -encoder3.get_speed();
            motor_speeds[3] = -encoder4.get_speed();
            float count[] = {encoder1, encoder2, encoder3};
            float speed[] = {motor_speeds[0], motor_speeds[1], motor_speeds[2]};
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
            // motors[0] = motor_target[0] / motor_gain * 0.95 * INT16_MAX;
            // motors[1] = motor_target[1] / motor_gain * 0.95 * INT16_MAX;
            // motors[2] = motor_target[2] / motor_gain * 0.95 * INT16_MAX;
            // motors[3] = motor_target[3] / motor_gain * 0.95 * INT16_MAX;
            for(size_t i = 0; i < 4; i++){
                motors[i] = (motor_target[i] / motor_gain + motor_pid[i].process(motor_speeds[i], motor_target[i])) * 0.95 * INT16_MAX;
            }
            // motors[2] = (motor_target[2] / motor_gain + motor2_pid.process(motor2_speed, motor_target[2])) * 0.95 * INT16_MAX;
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
            std::array<uint8_t, 7> hina_state;
            hina_state[0] = 0x03;
            hina_state[1] = 0;
            for (size_t i = 0; i < 5; i++)
            {
                hina_state[1] |= ret.hina_state.lmtsw[i] << i;
            }
            for (size_t i = 0; i < 2; i++)
            {
                hina_state[2] |= ret.hina_state.cylinder[i] << i;
            }
            for (size_t i = 0; i < 1; i++)
            {
                memcpy(&hina_state[3+4*i], &ret.hina_state.potentiometer[i], 4);
            }
            auto encoded_data2 = cobs_encode(hina_state);
            serial.write(encoded_data2.data(), encoded_data2.size());
            std::array<uint8_t, 5> debug_data;
            debug_data[0] = 0xff;
            memcpy(&debug_data[1], &ret.debug_data[0], 2);
            memcpy(&debug_data[3], &ret.debug_data[1], 2);
            auto encoded_debug_data = cobs_encode(debug_data);
            serial.write(encoded_debug_data.data(), encoded_debug_data.size());
        }
    }
}

static void MX_TIM3_Init(void)
{

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 57;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim3);

}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

