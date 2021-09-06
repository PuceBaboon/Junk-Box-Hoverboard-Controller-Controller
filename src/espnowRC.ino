/*
 *   $Id: espnowRC.ino,v 1.32 2021/09/02 00:27:28 gaijin Exp $
 *
 *  This is the "Remote Control Transmitter" half of the HoverBoard driver.  It
 *  provides a very simple, multi-switch (ie:- inc/dec control, -not- ADC) controller
 *  for H/B speed and direction.
 *
 *  The physical layout of the hand-held unit is (currently) two clusters (one
 *  on the L/H side, the other on the right) with the buttons laid out in a
 *  number-5 domino face for each cluster.  The ESP32 running the whole show
 *  is located at the top of the unit between the two clusters (it is at the 
 *  the top to keep the antenna clear of the PCB copper). 
 *
 *  The L/H side cluster is the traction speed/direction control and the R/H side
 *  is the blade motor speed/direction control (this may need supplementing with
 *  more buttons, depending upon how many blade motors we finally decide upon).
 *  In both cases, the centre button in the cluster is the "stop" button.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <WiFi.h>
#include <esp_wifi.h>           // *** FOR REFERENCE *** Important info in this file.
#include <esp_now.h>            // Note "esp_now.h" for the ESP32 ("espnow.h" for the 8266).
#include <esp_crc.h>		// CRC routines for the ESP-Now packet checksum.
#include <JC_Button_ESP.h>	// Jack Christiansen's debounce handler for Espressif devices.
#include <utils.h>		// LOCAL: Support utils.

/* *INDENT-OFF* */
extern "C" {
    #include <espnow_common.h>
    #include <espnowRC.h>	// Required for both espnowRC and espnowHB.
}
/* *INDENT-ON* */ 


// ESP-Now.
volatile boolean haveRecStat = false;		// ESP-Now received data call-back flag.
volatile boolean haveSndStat = false;		// ESP-Now sent data call-back flag.
volatile esp_now_send_status_t send_Stat;
bool ledOn = false;

#define TIME_SEND 100;				// FOR TESTING, ONLY.
unsigned long iTimeSend = 0;			// FOR TESTING, ONLY.
uint16_t ltick_cnt = 0;				// FOR TESTING, ONLY.

uint16_t cmd_seq_last = 0;			// Counter for command packet sequence numbers.



/* Instantiate buttons. */
Button L_TOP(BL_TOP, 25, 0, 1);
Button L_LEFT(BL_LEFT, 25, 0, 1);
Button L_CENT(BL_CENT, 25, 0, 1);
Button L_BOTT(BL_BOTT, 25, 0, 1);
Button L_RIGHT(BL_RIGHT, 25, 0, 1);

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02X:%02X:%02X:%02X:%02X:%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

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
  memcpy(&peerInfo.peer_addr, HB_ContAddr, 6);		// Set remote peer address.
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
  //           if (esp_now_send(HB_ContAddr, bs, sizeof(sensorData)) != ESP_NOW_SEND_SUCCESS) {
  retval = esp_now_send(peerInfo.peer_addr, (const uint8_t *) &cmnd_struc, sizeof(cmnd_struc));
  if (retval != ESP_OK) {
    DisplayESPNError(retval);
  }
}


void BtnHandler() {
//  const uint8_t LED_PIN(13);
//  static bool ledState;       // a variable that keeps the current LED status

  L_CENT.read();
  if (L_CENT.wasPressed()) {
    Serial.println("L/H CENT press");
  }
  if (L_CENT.wasReleased())
  {
    CmdSend(STOP_ALL);
    Serial.println("STOP_ALL");
  }

  L_TOP.read();
  if (L_TOP.wasPressed()) {
    Serial.println("Got L/H TOP press");
  }
  if (L_TOP.wasReleased())
  {
    CmdSend(SPEED_UP);
    Serial.println("SPEED_UP");
  }

  L_BOTT.read();
  if (L_BOTT.wasPressed()) {
    Serial.println("Got L/H BOTT press");
  }
  if (L_BOTT.wasReleased())
  {
    CmdSend(SLOW_DOWN);
    Serial.println("SLOW_DOWN");
  }

  L_LEFT.read();
  if (L_LEFT.wasPressed()) {
    Serial.println("Got L/H LEFT press");
  }
  if (L_LEFT.wasReleased())
  {
    CmdSend(STEER_LEFT);
    Serial.println("STEER_LEFT");
  }

  L_RIGHT.read();
  if (L_RIGHT.wasPressed()) {
    Serial.println("Got L/H RIGHT press");
  }
  if (L_RIGHT.wasReleased())
  {
    CmdSend(STEER_RIGHT);
    Serial.println("STEER_RIGHT");
  }
}


/*
 * Main loop section.
 */
void loop() {
  unsigned long timeNow = millis();

  /*
   * Highest priority tasks first.
   * Check the ESP-Now Send/Receive status and
   * handle data handed off by call-backs.
   */
  if (haveSndStat == true) {
    haveSndStat == false;
    Serial.print("  Send: ");
  }

  if (haveRecStat == true) {
    haveRecStat == false;
    Serial.print("  Rcv: ");
  }

  BtnHandler();

  // Limit command updates to (default) 100ms intervals.
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;

  /*
   * Note that local-tick_cnt is only updated if we
   * get this far (ie:- ~100ms intervals, depending
   * on TIME_SEND setting).
   */
  ltick_cnt++;

  // Events timed at very roughly every 1 second.
  if (ltick_cnt % 10 == 0) {

    /*
     * Send our heartbeat signal at one second
     * intervals.  If the HB-unit misses two consecutive
     * heartbeats, it will stop the mower and shut down
     * the cutters.
     */
    CmdSend(HEARTBT);
  }

  // Events timed at very roughly every 10 seconds.
//  if (ltick_cnt % 100 == 0) {
//    broadcast("BCST: From RC-unit!");
//    Serial.println("Sending broadcast...");
//  }

  // Blink the LED
  if (ltick_cnt % 1000 == 0) {
    digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  }
}


/* 
 * This is our receive call-back routine (which was an in-line lambda
 * function in earlier versions).  It runs when data is received from
 * a peer and really should be as concise as possible.
 */
void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
  // only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);

  // make sure we are null terminated
  buffer[msgLen] = 0;

  // format the mac address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);

  // debug log the message to the serial port
  Serial.printf("Received message from: %s - %s\r\n", macStr, buffer);

  // what are our instructions
  if (strcmp("on", buffer) == 0)
  {
    ledOn = true;
  }
  else if (strcmp("off", buffer) == 0)
  {
    ledOn = false;
  }
  else
  {
    Serial.println("Unknown command.");
  }
  digitalWrite(2, ledOn);
}


/*
 * This is the so-called "send call-back", but is actually called after
 * the data is sent (so should really be the "sent call-back).  It gives
 * us an indication of whether the data has actually been delivered, or
 * not (the actual esp_now_send() routine just gives us the return code).
 *
 * To test this, send data while the peers are all switched off.  The
 * esp_now_send() routine will return with ESP_OK, but this call-back
 * will flag delivery status as "Failed".
 */
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
  char macStr[18];

  formatMacAddress(macAddr, macStr, 18);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: Delivery ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successful" : "Failed");
}


/*
 * Send a packet to the RC-unit, using ESP-Now.
 */
void ESPNow_Send(){
}
/*
void dillyESPNow_Send(){
  sensorData.loc_id = 69;
  sensorData.temp = ((uint8_t) 68);
  sensorData.humidity = ((uint8_t) 67);
  sensorData.pressure = ((uint8_t) 66);

  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  if (esp_now_send(HB_ContAddr, bs, sizeof(sensorData)) != ESP_NOW_SEND_SUCCESS) {
    Serial.println("RC: !! Send status: Failed !!");
  } else {
    Serial.println("RC: Send status: Okay.");
  }
}
*/


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
 * Currently called "broadcast()", but actually it's just an
 * ESP-Now send routine.  Needs to be renamed and cleaned up.
 */
void broadcast(const String &message)
{
  esp_err_t retval = 0;

  /*
   * Set remote peer (or broadcast) address.
   */
  // uint8_t peerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// Broadcast (to all peers, including self).
  uint8_t peerAddress[] = {0x3C, 0x71, 0xBF, 0x9D, 0xEB, 0x34};	// Hoverboard Controller Unit.
  // uint8_t peerAddress[] = {0x3C, 0x71, 0xBF, 0x9E, 0x28, 0x94};		// RC-Unit.
  esp_now_peer_info_t peerInfo = {};
  memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));			// Clear out any cruft.
  memcpy(&peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = ESPNOW_DEFCHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(peerAddress))
  {
    retval = esp_now_add_peer(&peerInfo);
    if (retval != ESP_OK) {
      DisplayESPNError(retval);
    }
  }
  retval = esp_now_send(peerAddress, (const uint8_t *)message.c_str(), message.length());
  if (retval != ESP_OK) {
    DisplayESPNError(retval);
  }
}


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n\nRemote-Control ESP-Now Node v1.0\n\n");

  //Set device in STA mode to begin with.
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Example");

  // Output my MAC address - useful for later.
  Serial.print("My MAC Address is: ");
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

  Serial.println("Initializing L/H button cluster...");
  L_TOP.begin();
  L_LEFT.begin();
  L_RIGHT.begin();
  L_BOTT.begin();
  L_CENT.begin();

  Serial.println();		// This indicates the end of setup().
}
