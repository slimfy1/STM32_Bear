#include "dfplayer.h"
extern UART_HandleTypeDef huart1;

#define FEEDBACK 0x00

uint8_t mp3[10] = {0x7E, 0xFF, 06, 0x12, 00, 00, 00, 0xFE, 0xE9, 0xEF};
//uint8_t mp1[10] = {0x7E, 0xFF, 06, 0x12, 00, 00, 01, 0xFE, 0xE8, 0xEF};
uint8_t df_serial_message[10];
char hexString[20];

uint16_t checksum;

void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
    uint16_t Checksum = VERSION + CMD_LEN + cmd + FEEDBACK + Parameter1 + Parameter2;
    Checksum = 0-Checksum;

    uint8_t CmdSequence[10] = { START_BYTE, VERSION , CMD_LEN, cmd, FEEDBACK, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), END_BIT};

    HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_Init (uint8_t volume)
{
    Send_cmd(0x3F, 0x00, SOURCES);
    HAL_Delay(200);
    Send_cmd(0x06, 0x00, volume);
    HAL_Delay(500);
}

void DF_MP3_Play(uint8_t mp3_numb)
{
    /*
    mp3[6] = mp3_numb;
	checksum = checksum - mp3[1] - mp3[2] - mp3[3] - mp3[4] - mp3[5] - mp3[6];
	mp3[7] = checksum >> 8;
	mp3[8] = checksum;
    HAL_UART_Transmit(&huart1, mp3, 10, 1000);
    HAL_Delay(100);
     */
    Send_cmd(0x12, 0, mp3_numb);
    HAL_Delay(1200);
    while(!READ_BIT(GPIOA->IDR, GPIO_IDR_IDR5)){ HAL_Delay(100);};
}