/*
 *   $Id: espnowRC.h,v 1.2 2021/08/05 03:32:15 gaijin Exp $
 *
 * Defines specific to the ESP-Now Remote Control Transmitter.
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ESPNOW_RC__
#define __ESPNOW_RC__

#ifndef LED_BUILTIN
#define LED_BUILTIN  2
#endif

/*
 * Define the hardware pin assignments for the button
 * clusters.
 */
const uint8_t
	/* L/H Cluster. */
	BL_CENT(34),
	BL_TOP(32),
	BL_BOTT(35),
	BL_LEFT(36),
	BL_RIGHT(39),
	/* R/H Cluster. */
	BR_CENT(26),
	BR_TOP(29),
	BR_BOTT(24),
	BR_LEFT(25),
	BR_RIGHT(22);


#endif		// ifndef __ESPNOW_RC__

#ifdef __cplusplus
}
#endif
