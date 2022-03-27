#ifndef INC_DFPLAYER_H_
#define INC_DFPLAYER_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdbool.h"

#define NEXT 			0x01 //1 command "Next"
#define	PREVIOUS 		0x02 //1 command "Previous"
#define PLAY_TRACK		0x03 //1 command "Play specified track"(NUM) 0-2999
#define VOL_UP			0x04 //1 command "Increase volume"
#define VOL_DWN			0x05 //1 command "Decrease volume"
#define VOLUME			0x06 //1 command "Set specified volume"  range :0-30
#define EQUALIZER		0x07 //1 command "Set Specified EQ" (0/1/2/3/4/5) Normal/Pop/Rock/Jazz/Classic/Base
#define	PLAY_REPEAT 	0x08 //1 command "Play loop the specified track
#define SOURCE			0x09 //1 command "Set Specified playback source"(0/1/2/3/4) U/TF/AUX/SLEEP/FLASH
#define STANDBY			0x0A //1 command "Enter into standby/low power loss"
#define NORMAL			0x0B //1 command "Normal working"
#define	RESET			0x0C //1 command "Reset module"
#define PLAYBACK		0x0D //1 command "Play"
#define	PAUSE			0x0E //1 command "Pause"
#define FOLDER  		0x0F //1 command "Play track in specified folder"
#define	VOL_ADJUST 		0x10 //1 command "Volume adjust set" {DH:Open volume adjust} {DL: set volume gain 0~31}
#define LOOP_ALL 		0x11 //1 command "Play loop all" {1:start repeat play} {0:stop play}2).
#define	MP3_FOLDER		0x12 //2 play specified track from MP3 folder	0-9999
#define	ADVERT 			0x13 //2 Commercials	0-9999
#define	FOLDER1000		0x14 //2 Allows you to play up to 1000 track from a single directory. Support 15 folder
#define	BACKGROUND		0x15 //2 stop playback, play background
#define	STOP			0x16 //2 Stop playback
#define FOLDER_LOOP		0x17 //4
#define SHUFFLE			0x18 //4
#define	SINGLE_REPEAT	0x19 //3	Set/unset current playing track in single repeat mode
#define DAC				0x1A //4	DAC Open/close
#define PLAYLIST		0x30 // Not supported by YX5200-24SS. Only works on YX6300-24SS
#define ADVERT2			0x25 //4 Multi-folder commercials command
#define	PLUG_IN 		0x3A //4
#define	PLUG_OUT 		0x3B //4
#define U_END_PLAY 		0x3C //2U device finished playing last track
#define TF_END_PLAY		0x3D //2TF device finished playing last track
#define	FLASH_END_PLAY 	0x3E //2STAY
#define	INIT 			0x3F //2Send initialization parameters 0 - 0x0F(each bit represent one device of the low-four bits)
#define	ERROR 			0x40 //2Returns an error, request retransmission
#define	REPLY 			0x41 //2Reply
#define	STATUS 			0x42 //2Query the current status
#define	QUERY_VOLUME	0x43 //2Query the current volume
#define	QUERY_EQ		0x44 //2Query the current EQ
#define	QUERY_PLAYMODE 	0x45 //2Query the current playback mode
#define	QUERY_VERSION 	0x46 //2Query the current software version
#define	QUERY_U_FILES 	0x47 //2Query the total number of TF card files
#define	QUERY_TF_FILES 	0x48 //2Query the total number of U-disk files
#define	QUERY_F_FILES	0x49 //2Query the total number of flash files
#define	QUERY_KEEPON 	0x4A //2Keep on
#define	QUERY_U_CUR		0x4B //2Queries the current track of U-Disk
#define	QUERY_TF_CUR 	0x4C //2Queries the current track of TF card
#define	QUERY_F_CUR 	0x4D //2Queries the current track of Flash

#define START_BYTE		0x7e //Start message bit
#define END_BIT			0xEF //End message bit
#define SOURCES         0x02  // TF CARD
#define DF_UART         &huart1 // Uart port
#define VERSION         0xFF
#define CMD_LEN         0x06

void DF_MP3_Play(uint8_t mp3_numb);
void DF_Init (uint8_t volume);
void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2);

#endif /* INC_DFPLAYER_H__ */
