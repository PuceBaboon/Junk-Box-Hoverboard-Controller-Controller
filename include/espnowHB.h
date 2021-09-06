/*
 *   $Id: espnowHB.h,v 1.5 2021/09/02 11:13:29 gaijin Exp $
 *
 * Defines specific to the ESP32 handling communications with the 
 * HoverBoard controller.
 */
#ifndef __ESPNOWHB__
#define __ESPNOWHB__

#ifndef LED_BUILTIN
#define LED_BUILTIN  2
#endif

#define HTBT_TMOUT	2	// How many missed "heartbeat" packets
				// before we do a controlled stop.
#define HTBT_FLASH	80	// Fast LED warning flash duration (ms).

/*
 * Data structures and defines specific to the ESP <--> HoverBoard
 * UART interface.
 */

/*
 * Structure for command data going -to- the HB-motherboard.
 *                =NOT= ESP-Now RELATED
 */
typedef struct{
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;          // Used in HBC_Send().

/* 
 * Structure for feedback data -from- the HB-motherboard. 
 *               =NOT= ESP-Now RELATED
 */
typedef struct{
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;        // Both of these are
SerialFeedback NewFeedback;     // used in HBC_Receive().

#endif	// __ESPNOWHB__
