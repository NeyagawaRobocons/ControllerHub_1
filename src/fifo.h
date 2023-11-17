#pragma once
#include <mbed.h>

template<typename TYPE, size_t FIFO_SIZE>
class fifo
{
private:
    std::array<TYPE, FIFO_SIZE> buffer;
    size_t index_read;
    size_t data_len;
public:
    fifo();
    void clear();
    int write(TYPE* data, size_t len);
    int read(TYPE* buf_ptr, size_t len);
    int skip(size_t len);
    TYPE peek(size_t index);
    size_t get_len();
    ssize_t search(TYPE val);
};

template<typename TYPE, size_t FIFO_SIZE>
fifo<TYPE, FIFO_SIZE>::fifo(){}

template<typename TYPE, size_t FIFO_SIZE>
void fifo<TYPE, FIFO_SIZE>::clear(){
    this->index_read = 0;
    this->data_len = 0;
    return;
}

template<typename TYPE, size_t FIFO_SIZE>
int fifo<TYPE, FIFO_SIZE>::write(TYPE* data, size_t len){
    if(FIFO_SIZE - this->data_len < len)return 1;
    for(size_t i = 0; i < len; i++){
        auto virtual_index = this->index_read + this->data_len + i;
        if(virtual_index >= FIFO_SIZE){
            this->buffer[virtual_index - FIFO_SIZE] = data[i];
        }else{
            this->buffer[virtual_index] = data[i];
        }
    }
    this->data_len += len;
    return 0;
}

template<typename TYPE, size_t FIFO_SIZE>
int fifo<TYPE, FIFO_SIZE>::read(TYPE* buf_ptr, size_t len){
    if(this->data_len < len)return 1;
    for(size_t i = 0; i < len; i++){
        auto virtual_index = this->index_read + i;
        if(virtual_index >= FIFO_SIZE){
            buf_ptr[i] = this->buffer[virtual_index - FIFO_SIZE];
        }else{
            buf_ptr[i] = this->buffer[virtual_index];
        }
    }
    this->index_read = this->index_read + len > FIFO_SIZE ? this->index_read + len - FIFO_SIZE : this->index_read + len;
    this->data_len -= len;
    return 0;
}

template<typename TYPE, size_t FIFO_SIZE>
int fifo<TYPE, FIFO_SIZE>::skip(size_t len){
    if(this->data_len < len)return 1;
    this->index_read = this->index_read + len > FIFO_SIZE ? this->index_read + len - FIFO_SIZE : this->index_read + len;
    this->data_len -= len;
    return 0;
}

template<typename TYPE, size_t FIFO_SIZE>
TYPE fifo<TYPE, FIFO_SIZE>::peek(size_t index){
    auto virtual_index = this->index_read + index;
    if(virtual_index >= FIFO_SIZE){
        return this->buffer[virtual_index % FIFO_SIZE];
    }else{
        return this->buffer[virtual_index];
    }
}

template<typename TYPE, size_t FIFO_SIZE>
size_t fifo<TYPE, FIFO_SIZE>::get_len(){
    return data_len;
}

template<typename TYPE, size_t FIFO_SIZE>
ssize_t fifo<TYPE, FIFO_SIZE>::search(TYPE val){
    for (size_t i = 0; i < this->data_len; i++)
    {
        if(this->peek(i) == val)return i;
    }
    return -1;
}
