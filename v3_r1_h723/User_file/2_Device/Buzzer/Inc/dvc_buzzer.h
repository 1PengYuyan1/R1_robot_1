#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "main.h"
#define TONE_C5   1911 // Do
#define TONE_E5   1517 // Mi
#define TONE_G5   1275 // Sol
#define TONE_C6   956  // High Do
#define TONE_ERR  2000 // 报警音

#define TONE_LOW   400  // 低沉
#define TONE_MID   800  // 中音
#define TONE_HIGH  1500 // 高亢
#define TONE_ALRT  2500 // 尖锐 (用于错误)

#define NOTE_G4  2551
#define NOTE_A4  2272
#define NOTE_B4  2024
#define NOTE_C5  1911
#define NOTE_D5  1703
#define NOTE_E5  1517
#define NOTE_F5  1432
#define NOTE_G5  1275
#define NOTE_A5  1136
#define NOTE_B5  1012
#define NOTE_C6  956

// 定义模式
typedef enum {
    BUZZER_OFF = 0,     // 0: 静音
    BUZZER_STARTUP,     // 1: 开机音
    BUZZER_DEVICE_LOST, // 2: 离线报警 (急促)
    BUZZER_EMERGENCY    // 3: 急停报警 (长鸣)
} Buzzer_Mode_t;



#ifdef __cplusplus
extern "C" {
#endif

    void Buzzer_Init(void);
    void Buzzer_Stop(void);

    void Buzzer_Play_Startup(void);     // 1. 开机音 (播完才会返回)
    void Buzzer_Play_DeviceLost(void);  // 2. 离线报警 (滴-停 一次)
    void Buzzer_Play_Emergency(void);   // 3. 急停报警 (长鸣-停 一次)

    void Buzzer_Play_FuKua(void);

#ifdef __cplusplus
};
#endif
#endif