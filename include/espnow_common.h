/*
 *   $Id: espnow_common.h,v 1.25 2021/09/04 13:12:00 gaijin Exp $
 *
 * Defines common to both gateway and nodes.  This should 
 * be copied verbatim to the node src directory (or linked).
 *
 */
#ifndef __ESPNOW_COMMON__
#define __ESPNOW_COMMON__

#define ESPNOW_DEFCHANNEL	9	// Fixed channel for our specific ESP-Now ops.
uint8_t rec_MAC[ESP_NOW_ETH_ALEN];	// Storage for received-from MAC address.
uint8_t snd_MAC[ESP_NOW_ETH_ALEN];	// Storage for sent-to MAC address.

/*
 * Packet-type identifiers.
 */
const uint16_t SFRM_CMD = 0xBEEF;	// Start frame value for ESP-Now command packets.
const uint16_t SFRM_HBS = 0xABCD;	// Start frame value for H/B status packets.

/*
 *  Define our ESP32's MACs here.
 */
const uint8_t HB_ContAddr[ESP_NOW_ETH_ALEN] = { 0x3C, 0x71, 0xBF, 0x9D, 0xEB, 0x34 };	// H/B Controller Unit (ESP32).
const uint8_t RC_ContAddr[ESP_NOW_ETH_ALEN] = { 0x3C, 0x71, 0xBF, 0x9E, 0x28, 0x94 };	// Remote Control for H/B (ESP32).
const uint8_t BrdCastAddr[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };	// Broadcast to everyone.
const uint8_t DiagRcvAddr[ESP_NOW_ETH_ALEN] = { 0x5C, 0xCF, 0x7F, 0x23, 0xF4, 0xC1 };	// Diagnostic ESP-Now receiver address.

/*
 * Remote control commands.
 */
typedef enum {
    HEARTBT,		// NON-COMMAND. Heartbeat signal.
    STOP_ALL,		// Drive and cutters.
    STEER_LEFT,
    STEER_RIGHT,
    CENTRE_STEER,	// Centre the steering.
    SPEED_UP,
    SLOW_DOWN,
    REVERSE,		// Toggle - Slow down, stop and reverse direction.
    STOP_CUTTERS,	// Toggle - Stop/Start all cutters (drive still active).
    SEL_CUTTER,
    CUTTER_SPD_UP,	// Increase speed on selected cutter head.    
    CUTTER_SPD_DN,	// Decrease speed on selected cutter head.    
} rc_btn_cmd;


/*
 * ESP-Now receive-callback packet buffer.
 */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];	// Sender's MAC address.
    uint8_t *data;			// Packet data.
    int data_len;			// Length of data.
} recv_cb_pkt_t;


/*
 * Data returned to the HB-controller (ESP32) from the HB-motherboard (STM32F).
 * This can be transferred back to the (ESP32) RC-unit for display via USB to
 * a laptop during testing.
 */
struct __attribute__ ((packed)) HB_STATUS {
  int16_t	comSteer;	// Steer after normalizing and mixing.
  int16_t	comSpeed;	// Speed after normalizing and mixing.
  int16_t	RPM_R;		// Measured RPM of right wheel.
  int16_t	RPM_L;		// Measured RPM of left wheel.
  int16_t	battV;		// Battery Voltage: Calibrated Battery Voltage * 100.
  int16_t	tempr;		// Temperature: Temperature in °C * 10.
} HBStatus;

/*
 * Data sent between HB-controller (ESP32) and RC-unit (ESP32) to
 * control speed and direction of the two BLDC motors attached to
 * the HB-motherboard.
 */
typedef struct __attribute__ ((packed)) RC_COMMAND {
  struct cdata {
    uint16_t	start_frm;	// Start frame: 0xBEEF.
    uint16_t	seq_num;	// Sequence number of (this) sent command.
    rc_btn_cmd	rc_cmd;		// Actual command from RC-unit.
  } d;
  uint16_t	pkt_crc;	// CRC checksum for packet.
} RCCommand;


/************************************* ESPNOW EXAMPLE CODE FOLLOWS *************************************************/

/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         // Broadcast or unicast ESPNOW data.
    uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     // Sequence number of ESPNOW data.
    uint16_t crc;                         // CRC16 value of ESPNOW data.
    uint32_t magic;                       // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   // Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         // Send unicast ESPNOW data.
    bool broadcast;                       // Send broadcast ESPNOW data.
    uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       // Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       // Delay between sending two ESPNOW data, unit: ms.
    int len;                              // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
} espnow_send_param_t;

#endif

