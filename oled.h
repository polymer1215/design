#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define OLED_CMD  0U
#define OLED_DATA 1U

extern u8 OLED_GRAM[144][8];

void OLED_ClearPoint(u8 x, u8 y);
void OLED_ColorTurn(u8 inverted);
void OLED_DisplayTurn(u8 rotated);
void OLED_WR_Byte(u8 data, u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 mode);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode);
void OLED_DrawCircle(u8 x, u8 y, u8 radius);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode);
void OLED_ShowString(u8 x, u8 y, const u8 *text, u8 size, u8 mode);
void OLED_ShowNum(u8 x, u8 y, u32 number, u8 length, u8 size, u8 mode);
void OLED_ShowChinese(u8 x, u8 y, u8 index, u8 size, u8 mode);
void OLED_ScrollDisplay(u8 count, u8 spacing, u8 mode);
void OLED_ShowPicture(
    u8 x, u8 y, u8 width, u8 height, const u8 bitmap[], u8 mode);
void OLED_Init(void);
bool OLED_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif
