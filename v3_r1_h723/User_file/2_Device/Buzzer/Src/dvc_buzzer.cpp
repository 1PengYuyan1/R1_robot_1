#include "dvc_buzzer.h"
#include "tim.h"

// 频率定义 (基于 240MHz/240 = 1MHz 计数频率)
// ARR = 1,000,000 / 频率(Hz)
#define TONE_C5   1911 // 523Hz
#define TONE_E5   1517 // 659Hz
#define TONE_G5   1275 // 784Hz
#define TONE_C6   956  // 1046Hz
#define TONE_4KHZ 250  // 4000Hz (典型最响频率)

/**
 * @brief 内部辅助函数：设置频率和占空比
 * @param arr 自动重装载值 (决定频率)
 * @param on  1为发声, 0为静音
 */
static void Set_Tone(uint16_t arr, uint8_t on) {
    if (on == 0 || arr == 0) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // 停止震动
    } else {
        __HAL_TIM_SET_AUTORELOAD(&htim12, arr);           // 设置频率
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, arr / 2); // 50%占空比最响
    }
}

// 初始化：必须启动 Channel 2
void Buzzer_Init(void) {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Set_Tone(0, 0);
}

// 手动静音
void Buzzer_Stop(void) {
    Set_Tone(0, 0);
}

// ---------------------------------------------------------
// 1. 开机音: Do-Mi-Sol-Do
// ---------------------------------------------------------
void Buzzer_Play_Startup(void) {
    Set_Tone(TONE_C5, 1); HAL_Delay(150);
    Set_Tone(TONE_E5, 1); HAL_Delay(150);
    Set_Tone(TONE_G5, 1); HAL_Delay(150);
    Set_Tone(TONE_C6, 1); HAL_Delay(150);
    Buzzer_Stop();
}

// ---------------------------------------------------------
// 2. 离线报警: 使用 4000Hz 典型频率，更刺耳
// ---------------------------------------------------------
void Buzzer_Play_DeviceLost(void) {
    Set_Tone(TONE_4KHZ, 1); HAL_Delay(100);
    Buzzer_Stop();          HAL_Delay(100);
}

// ---------------------------------------------------------
// 3. 急停报警: 长鸣
// ---------------------------------------------------------
void Buzzer_Play_Emergency(void) {
    Set_Tone(TONE_4KHZ, 1); HAL_Delay(1000);
    Buzzer_Stop();          HAL_Delay(200);
}

void Buzzer_Play_FuKua(void) {
    // 似 (C5) 木 (C5) 头 (C5)
    Set_Tone(NOTE_C5, 1); HAL_Delay(200);
    Set_Tone(NOTE_C5, 1); HAL_Delay(200);
    Set_Tone(NOTE_C5, 1); HAL_Delay(400);
    Buzzer_Stop(); HAL_Delay(50);

    // 似 (C5) 木 (C5) 头 (C5)
    Set_Tone(NOTE_C5, 1); HAL_Delay(200);
    Set_Tone(NOTE_C5, 1); HAL_Delay(200);
    Set_Tone(NOTE_C5, 1); HAL_Delay(400);
    Buzzer_Stop(); HAL_Delay(50);

    // 不 (B4) 动 (A4)
    Set_Tone(NOTE_B4, 1); HAL_Delay(300);
    Set_Tone(NOTE_A4, 1); HAL_Delay(600);

    // 或者是 (A4) 我 (B4)
    Set_Tone(NOTE_A4, 1); HAL_Delay(200);
    Set_Tone(NOTE_B4, 1); HAL_Delay(200);

    // 实在 (C5) 太 (D5)
    Set_Tone(NOTE_C5, 1); HAL_Delay(200);
    Set_Tone(NOTE_D5, 1); HAL_Delay(200);

    // 浮 (E5) 夸 (G5) ——
    Set_Tone(NOTE_E5, 1); HAL_Delay(400);
    Set_Tone(NOTE_G5, 1); HAL_Delay(800);

    Buzzer_Stop();
}