#ifndef DATATYPE_H
#define DATATYPE_H

#include <cstdint>
#include <cstring>
#include <cmath>

bool Check_Encoder_Status(uint8_t status_byte);
uint32_t Convert_Position_To_Float(uint32_t raw_position);

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// 辅助函数
void FloatToBytes(float value, uint8_t *bytes);
float BytesToFloat(const uint8_t *bytes);

float ConvertToDegrees(uint32_t position);
float ConvertToRadians(uint32_t position);

#endif