#pragma once
#include <mbed.h>

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
