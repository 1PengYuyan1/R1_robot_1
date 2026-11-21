#include "DataType.h"

/// @brief 将浮点数转换为无符号整数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/// @brief 将原始位置值转换为浮点数位置值
uint32_t Convert_Position_To_Float(uint32_t raw_position)
{
    // 根据编码器分辨率将原始值转换为实际角度或位置值
    // 这里需要根据具体编码器规格进行调整
    return (float)raw_position;
}
// 检查编码器状态
bool Check_Encoder_Status(uint8_t status_byte)
{
    // 检查状态字节，返回true表示正常，false表示故障
    return (status_byte & 0x01) == 0; // 假设bit0为错误标志
}

// 辅助函数: 浮点数转字节数组
void FloatToBytes(float value, uint8_t *bytes)
{
    union
    {
        float f;
        uint32_t i;
    } converter;

    converter.f = value;

    bytes[0] = converter.i & 0xFF;
    bytes[1] = (converter.i >> 8) & 0xFF;
    bytes[2] = (converter.i >> 16) & 0xFF;
    bytes[3] = (converter.i >> 24) & 0xFF;
}

// 辅助函数: 字节数组转浮点数
float BytesToFloat(const uint8_t *bytes)
{
    union
    {
        float f;
        uint32_t i;
    } converter;

    converter.i = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
    return converter.f;
}

float ConvertToDegrees(uint32_t position) {
    // 根据编码器的分辨率进行转换
    // 假设编码器是32位绝对值编码器，一圈对应2^32个脉冲
    return (position * 360.0f) / 4294967296.0f;  // 2^32 = 4294967296
}

float ConvertToRadians(uint32_t position) {
    // 弧度转换：2π弧度 = 一圈
    return (position * 2.0f * M_PI) / 4294967296.0f;
}