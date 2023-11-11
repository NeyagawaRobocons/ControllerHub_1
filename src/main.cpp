#include <mbed.h>

#define debug

template <size_t N>
std::array<uint8_t, N+2> cobs_encode(std::array<uint8_t, N> input){
    std::array<uint8_t, N+2> encoded_data;
    encoded_data.fill(0x00);
    size_t index_last_zero = 0;
    for(size_t i=0; i < input.size(); i++){
        if(input[i] == 0x00){
            encoded_data[index_last_zero] = i + 1 - index_last_zero;
            index_last_zero = i + 1;
        }else{
            encoded_data[i+1] = input[i];
        }
    }
    encoded_data[index_last_zero] = input.size() + 1 - index_last_zero;
    return encoded_data;
}

template <size_t N>
std::array<uint8_t, N-2> cobs_decode(std::array<uint8_t, N> input){
    std::array<uint8_t, N-2> decoded_data;
    size_t next_zero = input[0];
    for(size_t i = 0; i < input.size() - 2; i++){
        next_zero--;
        if(next_zero == 0){
            decoded_data[i] = 0x00;
            next_zero = input[i + 1];
        }else{
            decoded_data[i] = input[i + 1];
        }
    }
    return decoded_data;
}

class encoder {
public:
    encoder(PinName pin_a, PinName pin_b, us_timestamp_t calc_period) : encoder_a(pin_a), encoder_b(pin_b), ticker(){
        encoder_a.rise(callback(this, &encoder::rise_a));
        encoder_b.rise(callback(this, &encoder::rise_a));
        encoder_a.fall(callback(this, &encoder::fall_a));
        encoder_b.fall(callback(this, &encoder::fall_a));
        this->ticker.attach_us(callback(this, &encoder::calculate_speed), calc_period);
        count = 0;
        speed = 0;
        this->calc_period = calc_period;
    }
    void reset(){
        count = 0;
    }
    operator int32_t(){
        return count;
    }
    //return speed in count per second with 4bit fraction
    int16_t get_speed(){
        return speed;
    }
private:
    void rise_a(){
        if(encoder_b.read()){
            count++;
        }else{
            count--;
        }
    }
    void rise_b(){
        if(encoder_a.read()){
            count--;
        }else{
            count++;
        }
    }
    void fall_a(){
        if(encoder_b.read()){
            count--;
        }else{
            count++;
        }
    }
    void fall_b(){
        if(encoder_a.read()){
            count++;
        }else{
            count--;
        }
    }
    void calculate_speed(){
        speed = (count - prev_count) * 16 * (1e6 / calc_period); // 16 is 4bit fraction
        prev_count = count;
    }
    InterruptIn encoder_a;
    InterruptIn encoder_b;
    Ticker ticker;
    us_timestamp_t calc_period;
    int32_t prev_count;
    int32_t count;
    int16_t speed;
};

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

static DigitalOut led(LED1);

int main(){
    #ifndef debug
    BufferedSerial serial(CONSOLE_TX, CONSOLE_RX, 115200);
    #endif
    encoder encoder1(PC_12, PC_13, 100e3);
    encoder encoder2(PC_14, PC_15, 100e3);
    encoder encoder3(PC_0, PC_1, 100e3);
    CAN can(PA_11, PA_12, 1e6);
    CANMessage msg;
    while(1){
        int16_t data[] = {8000, -8000, 8000, -8000};
        msg.id = 0x01;
        msg.len = 8;
        for(int i = 0; i < 4; i++){
            msg.data[2*i] = data[i] >> 8;
            msg.data[2*i+1] = data[i] & 0xff;
        }
        can.write(msg);
        led = !led;

        int32_t count[] = {encoder1, encoder2, encoder3};
        int16_t speed[] = {encoder1.get_speed(), encoder2.get_speed(), encoder3.get_speed()};
        data_array data2(count, speed);
        auto encoded_data = cobs_encode(data2.pack());
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
        wait_us(200e3);
    }
}
