#pragma once
#include <mbed.h>

#include "fifo.h"
#include "COBS.h"

template <size_t HEADER_SIZE>
struct data_type {
    array<uint8_t, HEADER_SIZE> header;
    size_t data_size;
};

template <size_t HEADER_SIZE, size_t RX_DATA_TYPE_NUM, size_t RX_BUFFER_SIZE>
class PackedSerial {
public:
    // @param serial: serial to use
    // @param rx_data_types: data_types to receive
    PackedSerial(BufferedSerial* serial, std::array<data_type<HEADER_SIZE>, RX_DATA_TYPE_NUM> rx_data_types);

    // @param header: header of data to send
    // @param data: data to send
    template <size_t TX_BUFFER_SIZE>
    void send(std::array<uint8_t, HEADER_SIZE> header, std::array<uint8_t, TX_BUFFER_SIZE> data);

    // @param rx_data_type: data_type of received data
    // @param data: received data
    // @return: 1 if received data is valid, 0 if not, -1 if error
    int receive(struct data_type<HEADER_SIZE>& rx_data_type, std::array<uint8_t, RX_BUFFER_SIZE>& data);
private:
    ssize_t index_from_header(array<uint8_t, HEADER_SIZE> header){
        for(size_t i = 0; i < RX_DATA_TYPE_NUM; i++){
            if(header == rx_data_types[i].header){
                return i;
            }
        }
        return -1;
    }
    BufferedSerial* serial;
    std::array<data_type<HEADER_SIZE>, RX_DATA_TYPE_NUM> rx_data_types;
    fifo<uint8_t, RX_BUFFER_SIZE> rx_buffer;
};

template <size_t HEADER_SIZE, size_t RX_DATA_TYPE_NUM, size_t RX_BUFFER_SIZE>
PackedSerial<HEADER_SIZE, RX_DATA_TYPE_NUM, RX_BUFFER_SIZE>::PackedSerial(BufferedSerial* serial, std::array<data_type<HEADER_SIZE>, RX_DATA_TYPE_NUM> rx_data_types) : serial(serial), rx_data_types(rx_data_types), rx_buffer() {
    
}

template <size_t HEADER_SIZE, size_t RX_DATA_TYPE_NUM, size_t RX_BUFFER_SIZE>
template <size_t TX_BUFFER_SIZE>
void PackedSerial<HEADER_SIZE, RX_DATA_TYPE_NUM, RX_BUFFER_SIZE>::send(std::array<uint8_t, HEADER_SIZE> header, std::array<uint8_t, TX_BUFFER_SIZE> data){
    std::array<uint8_t, HEADER_SIZE + TX_BUFFER_SIZE> packed_data;
    for(size_t i = 0; i < HEADER_SIZE; i++){
        packed_data[i] = header[i];
    }
    for(size_t i = 0; i < TX_BUFFER_SIZE; i++){
        packed_data[i + HEADER_SIZE] = data[i];
    }
    auto encoded_data = cobs_encode(packed_data);
    serial->write(encoded_data.data(), encoded_data.size());
}

template <size_t HEADER_SIZE, size_t RX_DATA_TYPE_NUM, size_t RX_BUFFER_SIZE>
int PackedSerial<HEADER_SIZE, RX_DATA_TYPE_NUM, RX_BUFFER_SIZE>::receive(struct data_type<HEADER_SIZE>& rx_data_type, std::array<uint8_t, RX_BUFFER_SIZE>& data){
    // read data to fifo
    std::array<uint8_t, RX_BUFFER_SIZE> buf;
    size_t read_len = this->serial->read(buf.data(), RX_BUFFER_SIZE - this->rx_buffer.get_len());
    if(read_len > 0){
        this->rx_buffer.write(buf.data(), read_len);
    }else{
        return 1;
    }
    // search packet and return error if there is notpacket
    size_t packet_len = this->rx_buffer.search(0x00u) + 1;
    if(packet_len == 0)return 1;
    if(packet_len < HEADER_SIZE + 2){this->rx_buffer.skip(packet_len); return 1;}
    // skip packet that is not match to database
    std::array<uint8_t, RX_BUFFER_SIZE> packet_buf;
    this->rx_buffer.read(packet_buf.data(), packet_len);
    auto decoded = cobs_decode(packet_buf);
    std::array<uint8_t, HEADER_SIZE> header;
    for (size_t i = 0; i < HEADER_SIZE; i++)
    {
        header[i] = decoded[i];
    }
    auto index = this->index_from_header(header);
    if(index == -1)return 1;
    if(this->rx_data_types[index].data_size != packet_len - HEADER_SIZE - 2)return 1;
    //return collect value
    rx_data_type = this->rx_data_types[index];
    for (size_t i = 0; i < packet_len - HEADER_SIZE - 2; i++)
    {
        data = decoded[i + HEADER_SIZE];
    }
    return 0;
}
