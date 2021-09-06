/*
 *   $Id: espnowHB.ino,v 1.65 2021/09/06 11:33:34 gaijin Exp $
 *
 * ESP32-based "receiver" for a HoverBoard-motor powered rover platform.
 * The "transmitter" part of this is a second ESP32 with a simple RC-style
 * switch cluster.
 *
 * The two ESP32s communicate using Espressif's proprietary ESP-Now
 * WiFi protocol (which uses the normal WiFi frequencies and channels, but
 * does away with TCP/UDP in favour of faster, but less robust, peer
 * to peer communications).
 *
 * This specific ESP32 (and this code) talks to the HoverBoard Controller
 * via a wired connection using UARTs on both sides and also talks to the
 * Remote Control unit (second ESP32) over that ESP-Now WiFi link.
 *
 * NOTE:- The HoverBoard interfacing parts of this code are copied verbatim
 *    from Emanuel's Arduino-serial (see header, below).
 *    Our version differs a little in that it DOES NOT use SoftwareSerial.
 *    We are using the UART2 hardware on an ESP32 instead.
 *
 * // *******************************************************************
 * //  Arduino Nano 5V example code
 * //  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
 * //
 * //  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
 * //
 * // *******************************************************************
 * // INFO:
 * // • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
 *
 *     >>>>  *** NOPE! THIS ESP32 VERSION USES THE UART2 HARDWARE, NOT SOFTWARESERIAL. ***
 *
 * // • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
 * //   it is recommended to use the built-in Serial interface for full speed perfomace.
 * // • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
 * // 
 * // CONFIGURATION on the hoverboard side in config.h:
 * // • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
 * //   #define CONTROL_SERIAL_USART3
 * //   #define FEEDBACK_SERIAL_USART3
 * //   // #define DEBUG_SERIAL_USART3
 * // • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
 * //   #define CONTROL_SERIAL_USART2
 * //   #define FEEDBACK_SERIAL_USART2
 * //   // #define DEBUG_SERIAL_USART2
 * // *******************************************************************
 *
 */

#include <WiFi.h>
#include <esp_wifi.h>			// *** FOR REFERENCE *** Important info in this file.
#include <esp_now.h>			// Note "esp_now.h" for the ESP32 ("espnow.h" for the 8266).

/* *INDENT-OFF* */
extern "C" {
  #include <utils.h>			// LOCAL: Support utils.
  #include "espnow_common.h"		// LOCAL: ESP-Now for this application.
  #include "espnowHB.h"			// LOCAL: Specific to this hardware.
}
/* *INDENT-ON* */ 


// ESP-Now.
volatile bool haveRecData = false;		// ESP-Now received data call-back flag.
volatile bool haveSndStat = false;		// ESP-Now sent data call-back flag.
bool HTBTtimeout = false;			// ESP-Now heartbeat watchdog triggered flag.
bool HTBTrcvd = false;				// ESP-Now heartbeat received flag.
volatile esp_now_send_status_t send_Stat;
recv_cb_pkt_t en_rcv_data = {};			// ESP-Now received data buffer (from callback).

bool buttonDown = false;
bool ledOn = false;

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen);
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status);

/* void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02X:%02X:%02X:%02X:%02X:%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
} */



// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD	115200	// [-] Baud rate for Serial2 (used to communicate with the hoverboard).
#define SERIAL_BAUD		115200	// [-] Baud rate for built-in Serial (used for the Serial Monitor).
#define START_FRAME		0xABCD	// [-] Start frame definition for HB-cont to HB-motherboard packets.
//##ORIGINAL##  #define TIME_SEND		100	// [ms] Sending time interval.
#define TIME_SEND		100	// [ms] Sending time interval  [Slow things down!].
#define SPEED_MAX_TEST		300	// [-] Maximum speed for testing (original FOC code value = 300).
//#define DEBUG_RX			// [-] Debug received data. Prints all bytes to serial (comment-out to disable).
/*
 * Both speed and direction are controlled by signed 
 * 16-bit integer values between -1000 and +1000.
 */
#define SPEED_CHANGE		20	// [-] Value for speed-change increment/decrement.
#define REVERSE_SLOW		-50	// [-] Constant value for slow revesrsing.
#define REVERSE_FAST		-150	// [-] Constant value for faster reverse.

/* We use the ESP32 UART2 hardware for our connection to the H/B interface. */
/*              ===  THIS IS SPECIFIC TO THE ESP32.  ===                    */
#define HB_RX 16		// Receive pin on ESP32 connected to the H/B interface.
#define HB_TX 17		// Transmit pin on ESP32 connected to the H/B interface.

/* Global variables and structures */
unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;
bool test1_f = false;		// True = running test1 now.
bool U2pktavail_f = false;	// True = new UART2 data queued.
uint8_t idx = 0;		// Index for new data pointer.
uint16_t bufStartFrame;		// Buffer Start Frame.
uint16_t ltick_cnt = 0;		// Local tick for control inside loop.
byte *p;			// Pointer declaration for the new received data.
byte incomingByte;
byte incomingBytePrev;


// TESTING TESTING TESTING!!
uint16_t cmd_seq_last = 0;
/*
 * Accepts a button-command and sends it over ESP-Now to the
 * HB-unit remote peer.
 */
void CmdSend(rc_btn_cmd cmnd) {
  RCCommand cmnd_struc;
  esp_err_t retval = 0;

  cmnd_struc.d.start_frm	= SFRM_CMD;		// Add ESP-Now command start-frame ID.
  cmnd_struc.d.seq_num		= cmd_seq_last++;	// Add packet sequence number.
  cmnd_struc.d.rc_cmd		= cmnd;			// Insert actual command.
  
/*---------TO-DO------------TO-DO------------TO-DO----------TO-DO--------
 *
 * @brief CRC16 value in little endian.
 *
 * @param crc: Initial CRC value (result of last calculation or 0 for the first time)
 * @param buf: Data buffer that used to calculate the CRC value
 * @param len: Length of the data buffer
 * @return CRC16 value
 *
 * static inline uint16_t esp_crc16_le(uint16_t crc, uint8_t const *buf, uint32_t len)
 * {
 *     return crc16_le(crc, buf, len);
 * }
 *  cmnd_struc.pkt_crc	= esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);
 *--------------------------------------------------------------------*/

  esp_now_peer_info_t peerInfo = {};			// Create new ESP-Now message struct.
  memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));	// Clear out any cruft.
  memcpy(&peerInfo.peer_addr, RC_ContAddr, 6);		// Set remote peer address.
  peerInfo.channel = ESPNOW_DEFCHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(peerInfo.peer_addr))	// Check peer exists in peer table.
  {
    retval = esp_now_add_peer(&peerInfo);
    if (retval != ESP_OK) {
      DisplayESPNError(retval);
    }
  }
  retval = esp_now_send(peerInfo.peer_addr, (const uint8_t *) &cmnd_struc, sizeof(cmnd_struc));
  if (retval != ESP_OK) {
    DisplayESPNError(retval);
  }
}


/*
 * Receive call-back (short version).
 * Copy packet into buffer and set flag.  Processing of saved
 * data done in loop().
 * Loads data into en_rcv_data (global) buffer.
 */
void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
  if (macAddr == NULL || data == NULL || dataLen <= 0) {
    Serial.println("RecvCB: NULL Data Error.");
    return;
  }

  /* Save received data into recv_cb buffer. */
  memcpy(&en_rcv_data.mac_addr, macAddr, ESP_NOW_ETH_ALEN);
/*  en_rcv_data.data = (uint8_t *) calloc(1, dataLen);
  if (en_rcv_data.data == NULL) {
    Serial.println("RecvCB: Data Calloc Error.");
    return;
  }
  if (memcpy(&en_rcv_data.data, data, dataLen) == (void *) NULL) {
    Serial.println("RcvCB: Memcpy Failure.");
    return;
  }
*/
  en_rcv_data.data = (uint8_t *) data;
  en_rcv_data.data_len = dataLen;
  haveRecData = true;
}


/*
 * This is the so-called "send call-back", but is actually called after
 * the data is sent (so should really be the "sent call-back").  It gives
 * us an indication of whether the data has actually been delivered, or
 * not (the actual esp_now_send() routine just gives us the return code).
 *
 * To test this, send data while the peers are all switched off.  The
 * esp_now_send() routine will return with ESP_OK, but this call-back
 * will flag delivery status as "Failed".
 */
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.print("Packet Sent To: ");
  printMacAddress((uint8_t *)macAddr);
  Serial.print("Packet Sent Status: Delivery ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successful." : "Failed.");
}


/*
 * Display ESP-Now specific errors.
 *
 * This is a crock and needs to be updated every time that
 * there are more errors added to the SDK.
 */
void DisplayESPNError(esp_err_t retval) {
  switch (retval) {
    case ESP_OK:
      Serial.println("Send returned success");
      break;
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("ESPNOW not Init.");
      break;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("Invalid Argument");
      break;
    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("Internal Error");
      break;
    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("Out of Memory");
      break;
    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("Peer not found.");
      break;
    case ESP_ERR_ESPNOW_FULL:
      Serial.println("Peer Table Full.");
      break;
    case ESP_ERR_ESPNOW_EXIST:
      Serial.println("Peer Already Exists.");
      break;
    case ESP_ERR_ESPNOW_IF:
      Serial.println("Interface Error.");
      break;
    default:
      Serial.println("Unknown error: Wierd shit just happened!");
  }
}


/*
 * Send data over UART2 to the HoverBoard Controller (-not- to 
 * the Remote Control unit).
 *
 * uSteer and uSpeed are both -signed- integers. Varying them
 * between -1000 and +1000 will provide reverse/forward on uSpeed
 * and left/right on uSteer. 
 */
void HBC_Send(int16_t uSteer, int16_t uSpeed) {

  /* Belt & braces check for out-of-range speed requests. */
  if (uSpeed > SPEED_MAX_TEST) uSpeed = SPEED_MAX_TEST;
  if (uSpeed < -SPEED_MAX_TEST) uSpeed = -SPEED_MAX_TEST;
  if (uSteer > SPEED_MAX_TEST) uSteer = SPEED_MAX_TEST;
  if (uSteer < -SPEED_MAX_TEST) uSteer = -SPEED_MAX_TEST;

  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to UART2.
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}


/*
 * Receive data over UART2 from the HoverBoard Controller (-not-
 * to the Remote Control unit).
 *
 * =================================================================
 *
 *    == TO-DO ==
 *
 *         The print-out of received packets from the HB-motherboard
 *         needs to be redirected to the RC-unit via espnow, as the 
 *         USB port on the HB-controller ESP32 is unusable, due to
 *         the power delivery for that ESP32 already coming from the
 *         HB-motherboard +15v supply.
 *
 * =================================================================
 *
 */
void HBC_Receive() {
  // Check for new data availability in the Serial buffer
  if (Serial2.available()) {
    incomingByte  = Serial2.read();					// Read the incoming byte.
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	// Construct the start frame.
  } else {
    return;
  }

  // If DEBUG_RX is defined, print all incoming bytes.
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif

  // Copy received data.
  if (bufStartFrame == START_FRAME) {	          // Initialize if new data is detected.
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;	
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data.
    *p++  = incomingByte; 
    idx++;
  }	
  
  // Check whether we have reached the end of the package.
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
              ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data.
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data.
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      U2pktavail_f = true;	// Flag arrival of valid feedback packet.

      // Print data to built-in Serial  [ XXX TO-DO XXX -- This needs to be sent to the RCU via espnow, instead XXX ].
      Serial.print(" st-");  Serial.print(Feedback.cmd1);		// Steering.
      Serial.print(" sp-");  Serial.print(Feedback.cmd2);		// Speed.
      Serial.print(" sR-");  Serial.print(Feedback.speedR_meas);	// RPM Right.
      Serial.print(" sL-");  Serial.print(Feedback.speedL_meas);	// RPM Left.
      Serial.print(" V-");  Serial.print(Feedback.batVoltage);		// Battery voltage.
      Serial.print(" T-");  Serial.print(Feedback.boardTemp);		// Temperature.
      Serial.print(" L-");  Serial.println(Feedback.cmdLed);		// Side board LED. (?)
    } else {
      Serial.println("!! UART2: BAD PACKET !!");
    }
    idx = 0;  // Reset the index.
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}


/*
 * Heartbeat Watchdog Stop.
 *
 * This routine is called when we lose contact with the RC-unit for
 * more than HB_TIMEOUT seconds (see loop(), below).
 *
 * It will stop the cutter motors and bring the mower to a controlled
 * stop.
 */
void HBW_Stop() {
  HBC_Send(0, 0);
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n\nHoverBoard ESP-Now Node v1.0\n\n");

  // Initially set WiFi to STA mode.
  WiFi.mode(WIFI_STA);

  // Display our local MAC.
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
 
  esp_wifi_set_channel(ESPNOW_DEFCHANNEL, (wifi_second_chan_t) WIFI_SECOND_CHAN_NONE);  // Set our specific channel.

  /*
   * Disconnect WiFi and display current channel
   * before startin ESP-Now.
   */
  WiFi.disconnect();

  displayWiFiChan();

  Serial.println("Initializing ESP-NOW...");
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  } else {
    Serial.println("ESPNow Init Failed");
    delay(3000);
    ESP.restart();
  }

  /*
   * Use the ESP32's "Boot" button as input
   * and the built-in blue LED as output.
   */
  pinMode(0, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  /*
   * Initialize the hardware-serial handling for the
   * connection between the ESP32 and the HB-motherboard  
   * over (ESP32)UART2.
   */
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HB_RX, HB_TX);

  // test1_f = true;		// Run motor test once.

  Serial.println();		// This indicates the end of setup().

  /*
   * Send STOP on initial start of communications
   * with the HB-controller motherboard.
   */
  HBC_Send(0, 0);
}


/*
 * Note that this default test loop from FOC/Arduino/hoverserial.ino
 * doesn't use any delay() statements.  It instead uses return statements
 * together with "iTimeSend" and "timenow" counts to force the
 * loop() to restart, thus providing a simple-but-elegant, fast,
 * non-blocking processing loop which only sends commands on a fairly
 * accurate ms count (the default is 100ms).  Nice!
 */
int8_t hb_cnt = HTBT_TMOUT, biLED_stat = 0;
int16_t iSteer = 0, iSpeed = 0;
unsigned long oldtNow = 0;

void loop() {
  unsigned long timeNow = millis();


  // Check for new received data (from the HoverBoard Controller).
  HBC_Receive();

  // Check for ESP-Now Send/Receive status.
  if (haveSndStat == true) {
    Serial.print("  Send: ");
    haveSndStat = false;
  }

  /*
   * Process incoming ESP-Now data, if it is flagged
   * as available.
   */
  if (haveRecData == true) {
    
    Serial.print("Recv from: ");
    printMacAddress(en_rcv_data.mac_addr);

//    Serial.print("Reported Length: ");
//    Serial.println(en_rcv_data.data_len, DEC);
//    Serial.print("Actual Bytes: ");
//    Serial.println(sizeof(en_rcv_data.data), DEC);

//  TESTING!   TESTING!   TESTING!
RCCommand loc_bf = {};

// Serial.print("loc_bf struct Bytes: ");
// Serial.println(sizeof(RCCommand), DEC);
// for (uint8_t i=0; i<sizeof(en_rcv_data.data); i++) {
//   Serial.println(en_rcv_data.data[i], HEX);
// }

/* Clear buffer to all zeros. */
if (memset((void *) &loc_bf, 0, sizeof(RCCommand)) == NULL) {
  Serial.println("!! Memset Error !!");
  return;
}

/* Copy ESP-Now data into local buffer (struct). */
if (memcpy((void *) &loc_bf, en_rcv_data.data, sizeof(RCCommand)) == NULL) {
  Serial.println("!! Memcpy Error !!");
} else {
  /* Check start-frame and print received command. */
  if (loc_bf.d.start_frm != 0xBEEF) {
    Serial.println("Bad Command Packet Received.");
  } else {
    Serial.print("Received Command: ");
    switch(loc_bf.d.rc_cmd) {

     /*
      * HEARTBT is the heartbeat watchdog timer.
      * This received packet simply "feeds" the
      * watchdog.  The check for a watchdog trigger
      * is further down the loop() in the one-second
      * "tick" section.
      */
      case HEARTBT:   hb_cnt = HTBT_TMOUT;	// Reset the heartbeat watchdog count.
                      HTBTrcvd = true;		// Set heartbeaqt received flag.
                      Serial.println("Heart-Bt");
                      break;

      case STOP_ALL:  Serial.println("Stop!!");
                      iSpeed = iSteer = 0;
                      HBC_Send(iSteer, iSpeed);
                      break;
      case SPEED_UP:  Serial.println("Faster!");
                      iSpeed += SPEED_CHANGE;
                      HBC_Send(iSteer, iSpeed);
                      break;
      case SLOW_DOWN:  Serial.println("Slower!");
                      iSpeed -= SPEED_CHANGE;
                      HBC_Send(iSteer, iSpeed);
                      break;
      case STEER_LEFT:  Serial.println("Left!");
                      iSteer -= SPEED_CHANGE;
                      HBC_Send(iSteer, iSpeed);
                      break;
      case STEER_RIGHT:  Serial.println("Right!");
                      iSteer += SPEED_CHANGE;
                      HBC_Send(iSteer, iSpeed);
                      break;
      case REVERSE:   Serial.println("Reverse!");
                      iSteer = 0;  iSpeed = REVERSE_SLOW;
                      HBC_Send(iSteer, iSpeed);
                      break;
/*
 * Note to dumb self -- We're going to need a second HB-controller
 * motherboard for the dual cutter motors, so this very probably
 * is not going to go here at all.
 */
      case STOP_CUTTERS:   Serial.println("Stop Cutters!");
                      break;
      case SEL_CUTTER:   Serial.println("Cutter Select");
                      break;
      case CUTTER_SPD_UP:  Serial.println("Cutter Faster!");
                      break;
      case CUTTER_SPD_DN:  Serial.println("Cutter Slower!");
                      break;
      default:  Serial.println("Unknown Command.");
                      break;
    }
  }
}


    haveRecData = false;
  }

  /*
   * Fast flash of onboard LED to indicate loss of
   * contact with RC-unit.
   */
  if ((HTBTtimeout == true) & ((timeNow - oldtNow) >= HTBT_FLASH)) {
    biLED_stat = !biLED_stat;
    digitalWrite(LED_BUILTIN, biLED_stat);
    oldtNow = timeNow; 
  }

  /*
   * Send on UART2 feedback data to RC-unit (for now) via
   * ESPNow.
   */
  if (U2pktavail_f == true) ;;


  /*
   * !! WARNING !! Expcts TIME_SEND interval to be 100ms.
   *
   * Limit command updates to (default) 100ms intervals.
   */
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;

  /*
   *  *** HB-CONTROLLER MOTHERBOARD TIMEOUT HEARTBEAT ***
   *
   * We simply re-send the current values for steering
   * and speed to prevent the UART2 link from timing out
   * at the HB-controller end (nothing to do with the
   * ESP32 or ESP-Now).
   */
  HBC_Send(iSteer, iSpeed);

  /*
   * !! WARNING !! Expcts TIME_SEND interval to be 100ms.
   *
   * Note that ltick_cnt is only updated if we get
   * this far (ie:- ~100ms intervals, depending on
   * TIME_SEND setting).
   */
  ltick_cnt++;

  // Events timed at very roughly every 1 second.
  if (ltick_cnt % 10 == 0) {

    /*
     * Check and update the heartbeat watchdog.
     */
    if (!HTBTrcvd) {
      if (--hb_cnt < 1) {
        HTBTtimeout = true;	// Set timeout flag.
        HBW_Stop();
      }
    } else {
      HTBTrcvd = false;		// Reset received flag.
      HTBTtimeout = false;	// Reset timeout flag.
      /* Flash LED at normal, 1Hz. */
      biLED_stat = !biLED_stat;
      digitalWrite(LED_BUILTIN, biLED_stat);
    }

    /* Send our own heartbeat to the RC-unit. */
    CmdSend(HEARTBT);
  }

  // Events timed at very roughly every 10 seconds.
  if (ltick_cnt % 100 == 0) {
    ;;
  }
}
