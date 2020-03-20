#include "dfplayer.h"
extern UART_HandleTypeDef huart1;

uint8_t mp3[10] = {0x7E, 0xFF, 06, 0x12, 00, 00, 00, 0xFE, 0xE9, 0xEF};
//uint8_t mp1[10] = {0x7E, 0xFF, 06, 0x12, 00, 00, 01, 0xFE, 0xE8, 0xEF};

void mp3_play(uint8_t mp3_numb)
{
    mp3[6] = mp3[6]+mp3_numb;
    mp3[8] = mp3[8]-mp3_numb;
    HAL_UART_Transmit(&huart1, mp3, 10, 1000);
    HAL_Delay(100);
    mp3[6] = 0x00;
    mp3[8] = 0xE9;
}
