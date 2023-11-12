#pragma once
#include <mbed.h>

template <size_t N>
std::array<uint8_t, N+2> cobs_encode(std::array<uint8_t, N> input);

template <size_t N>
std::array<uint8_t, N-2> cobs_decode(std::array<uint8_t, N> input);
