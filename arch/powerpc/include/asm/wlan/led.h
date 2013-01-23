/*
 * file asm/wlan/led.h
 * copyright 2011 Siemens AG
 *
 * this file sets the led definitions for Siemens wlan support
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 2 of the License.
 *
 * This implements simple platform support for PowerPC 44x chips.  This is
 * mostly used for eval boards or other simple and "generic" 44x boards.  If
 * your board has custom functions or hardware, then you will likely want to
 * implement your own board.c file to accommodate it.
 */

/*********************************************************************
*	DESCRIPTION:
* 		Header file defining the supported LEDs
*
**********************************************************************
* 	FILENAME:
* 		ledpriv.h
*
*	VERSION
*		0.1
*
*	AUTHOR:
* 		Darshan Deshmukh
*
*	DATE:
*		04/01/2011
*
*	CHANGES:
*		--
*********************************************************************/

#ifndef LED_H_
#define LED_H_


/*********************************************************************
* LED Definitions
*********************************************************************/
/* Common LEDs */

#define LED_ERROR_RED		0x00000001


/* SCALANCE W Specific LEDs */

#define LED_ETH_GREEN		0x00000010
#define LED_ETH_YELLOW		0x00000020
#define LED_WLAN1_GREEN		0x00000040
#define LED_WLAN1_YELLOW	0x00000080
#define LED_WLAN2_GREEN		0x00000100
#define LED_WLAN2_YELLOW	0x00000200
#define LED_WLAN3_GREEN		0x00000400
#define LED_WLAN3_YELLOW	0x00000800
#define LED_POWER1_ON		0x00001000
#define LED_POWER2_ON		0x00002000
#define LED_POWER_POE_ON	0x00004000
#define LED_ETH_SFP1_GREEN	0x00008000
#define LED_ETH_SFP1_YELLOW	0x00010000
#define LED_ETH_SFP2_GREEN	0x00020000
#define LED_ETH_SFP2_YELLOW	0x00040000

#define LED_MAX				LED_ETH_SFP2_YELLOW

#if 0
/* SCALANCE W LEDs state's array index*/
#define LED_ERROR_RED_IND		0
#define LED_ETH_GREEN_IND		4
#define LED_ETH_YELLOW_IND		5
#define LED_WLAN1_GREEN_IND		6
#define LED_WLAN1_YELLOW_IND	7
#define LED_WLAN2_GREEN_IND		8
#define LED_WLAN2_YELLOW_IND	9
#define LED_WLAN3_GREEN_IND		10
#define LED_WLAN3_YELLOW_IND	11
#define LED_POWER1_ON_IND		12
#define LED_POWER2_ON_IND		13
#define LED_POWER_POE_ON_IND	14
#endif

/********************************************************************/

/* LED commands */
#define SET_LED_STATE  0 // command to switch LEDs on or off.
#define SET_LED_BLINK  1 // command to switch blink operation on or off.
#define SET_LED_FLASH  2 // command to let LEDs flash.
#define GET_LED_STATE  3 // command to get the actual LED state

/* LED states */
#define LED_OFF 		0 // switch LEDs off
#define LED_ON  		1 // switch LEDs on
#define BLINK_OFF		0 // switch the LED blinking off
#define BLINK_ON		1 // switch the LED blinking on



#define DFS_BLINK_TIME					1
#define CLIENT_AUTO_ADOPT_ON_BLINK_TIME	1
#define CLIENT_NOT_CONNECTED_BLINK_TIME 5

#define PACKET_FLASH_TIME				2

#define ATH_STA_AUTO_ADOPT_LED_BLINK_TIME		1 /* seconds */ * (1000 / HZ) * 1000

/* Kernel specific declarations */
#ifdef __KERNEL__

/* Kernel interface to set primary state of leds */
extern int ledSetLeds(unsigned int Command, unsigned long *LEDS,
		              unsigned long STATE, unsigned long *OFF,
		              unsigned char OnTime, unsigned char OffTime,
		              unsigned int Number, unsigned char FlashTime);

static inline void set_wlan_leds (int devno, int type, unsigned long state, unsigned char time)
{
	unsigned long led_array[4] = {0,0,0,0};

	switch (type) {
	case SET_LED_STATE:
		led_array[0] = LED_WLAN1_GREEN<<(devno*2);
		ledSetLeds(SET_LED_STATE,led_array,state,0,0,0,0,0);
		break;
	case SET_LED_BLINK:
		led_array[0] = LED_WLAN1_GREEN<<(devno*2);
		ledSetLeds(SET_LED_BLINK,led_array,state,0,time,time,0,0);
		break;
	case SET_LED_FLASH:
		{
			unsigned long led_on[4] = {0,0,0,0};
			unsigned long led_off[4] = {0,0,0,0};

			led_on[0] = LED_WLAN1_YELLOW << (devno*2);
			led_off[0] = LED_WLAN1_GREEN << (devno*2);

			ledSetLeds(SET_LED_FLASH,led_on,0,led_off,0,0,0,time);
		}
		break;
	}
}
#endif /* __KERNEL__ */


#endif /* LED_H_ */

#define ETH_LED_FLASH_TIME 100

/* inline ???? */
void scw_set_startup_led_state(unsigned long led, int state);
