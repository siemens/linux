//
// defines for SOC1 GPIO
//
// file "socGen.h"
// h.b. AD ATS 11 
// 06-March-2008
//

#ifndef _ASM_ARCH_GPIO_H
#define _ASM_ARCH_GPIO_H

/*--- #define-Konstanten und Makros ---------------------------------*/
#define SOC_GPIO_START          0xBFA00000  /* General Purpose I/O Base Addr */
#define SOC_GPIO_LENGTH         0x00100000  /* 1 MB */

#define GPIO_BLOCKS 6

/*--- GPIO Register for GPIO 0..31 ----------------------------------*/
#define GPIO_IOCTRL_0		0x00 /* conf: 0=output 1=input */
#define GPIO_OUT_0		0x04 /* output */
#define GPIO_OUT_SET_0		0x08 /* bit selective output: 0=unchanged 1= set 1 */
#define GPIO_OUT_CLEAR_0	0x0C /* bit selective output: 0=unchanged 1= set 0 */
#define GPIO_RES_DIS_0		0x10 /* bit selective reset disabling of XRESET_GPIO_SM */
#define GPIO_IN_0		0x14 /* input */
#define GPIO_PORT_MODE_0_L	0x18 /* function assignment */
#define GPIO_PORT_MODE_0_H	0x1C /* function assignment */

/*--- GPIO Register for GPIO 32..63 ----------------------------------*/
#define GPIO_IOCTRL_1		0x20
#define GPIO_OUT_1		0x24
#define GPIO_OUT_SET_1		0x28
#define GPIO_OUT_CLEAR_1	0x2C
#define GPIO_RES_DIS_1		0x30
#define GPIO_IN_1		0x34
#define GPIO_PORT_MODE_1_L	0x38
#define GPIO_PORT_MODE_1_H	0x3C

/*--- GPIO Register for GPIO 64..95 ----------------------------------*/
#define GPIO_IOCTRL_2		0x40
#define GPIO_OUT_2		0x44
#define GPIO_OUT_SET_2		0x48
#define GPIO_OUT_CLEAR_2	0x4C
#define GPIO_RES_DIS_2		0x50
#define GPIO_IN_2		0x54
#define GPIO_PORT_MODE_2_L	0x58
#define GPIO_PORT_MODE_2_H	0x5C

/*--- GPIO Register for GPIO 96..127 ----------------------------------*/
#define GPIO_IOCTRL_3		0x60
#define GPIO_OUT_3		0x64
#define GPIO_OUT_SET_3		0x68
#define GPIO_OUT_CLEAR_3	0x6C
#define GPIO_RES_DIS_3		0x70
#define GPIO_IN_3		0x74
#define GPIO_PORT_MODE_3_L	0x78
#define GPIO_PORT_MODE_3_H	0x7C

/*--- GPIO Register for GPIO 128..159 ----------------------------------*/
#define GPIO_IOCTRL_4		0x80
#define GPIO_OUT_4		0x84
#define GPIO_OUT_SET_4		0x88
#define GPIO_OUT_CLEAR_4	0x8C
#define GPIO_RES_DIS_4		0x90
#define GPIO_IN_4		0x94
#define GPIO_PORT_MODE_4_L	0x98
#define GPIO_PORT_MODE_4_H	0x9C

/*--- GPIO Register for GPIO 160..191 ----------------------------------*/
#define GPIO_IOCTRL_5		0xA0
#define GPIO_OUT_5		0xA4
#define GPIO_OUT_SET_5		0xA8
#define GPIO_OUT_CLEAR_5	0xAC
#define GPIO_RES_DIS_5		0xB0
#define GPIO_IN_5		0xB4
#define GPIO_PORT_MODE_5_L	0xB8
#define GPIO_PORT_MODE_5_H	0xBC

/******************************************************************************/
#define P0	0x00000001	/* GPIO Port 0/32/64/96/128/160  */
#define P1	0x00000002	/* GPIO Port 1/33/65/97/129/161  */
#define P2	0x00000004	/* GPIO Port 2/34/66/98/130/162  */
#define P3	0x00000008	/* GPIO Port 3/35/67/99/131/163  */
#define P4	0x00000010	/* GPIO Port 4/36/68/100/132/164  */
#define P5	0x00000020	/* GPIO Port 5/37/69/101/133/165  */
#define P6	0x00000040	/* GPIO Port 6/38/70/102/134/166  */
#define P7	0x00000080	/* GPIO Port 7/39/71/103/135/167  */
#define P8	0x00000100	/* GPIO Port 8/40/72/104/136/168  */
#define P9	0x00000200	/* GPIO Port 9/41/73/105/137/169  */
#define P10	0x00000400	/* GPIO Port 10/42/74/106/138/170  */
#define P11	0x00000800	/* GPIO Port 11/43/75/107/139/171  */
#define P12	0x00001000	/* GPIO Port 12/44/76/108/140/172  */
#define P13	0x00002000	/* GPIO Port 13/45/77/109/141/173  */
#define P14	0x00004000	/* GPIO Port 14/46/78/110/142/174  */
#define P15	0x00008000	/* GPIO Port 15/47/79/111/143/175  */
#define P16	0x00010000	/* GPIO Port 16/48/80/112/144/176  */
#define P17	0x00020000	/* GPIO Port 17/49/81/113/145/177  */
#define P18	0x00040000	/* GPIO Port 18/50/82/114/146/178  */
#define P19	0x00080000	/* GPIO Port 19/51/83/115/147/179  */
#define P20	0x00100000	/* GPIO Port 20/52/84/116/148/180  */
#define P21	0x00200000	/* GPIO Port 21/53/85/117/149/181  */
#define P22	0x00400000	/* GPIO Port 22/54/86/118/150/182  */
#define P23	0x00800000	/* GPIO Port 23/55/87/119/151/183  */
#define P24	0x01000000	/* GPIO Port 24/56/88/120/152/184  */
#define P25	0x02000000	/* GPIO Port 25/57/89/121/153/185  */
#define P26	0x04000000	/* GPIO Port 26/58/90/122/154/186  */
#define P27	0x08000000	/* GPIO Port 27/59/91/123/155/187  */
#define P28	0x10000000	/* GPIO Port 28/60/92/124/156/188  */
#define P29	0x20000000	/* GPIO Port 29/61/93/125/157/189  */
#define P30	0x40000000	/* GPIO Port 30/62/94/126/158/190  */
#define P31	0x80000000	/* GPIO Port 31/63/95/127/159/191  */

#ifdef CONFIG_AuD_SOC1_CP1500
#if (CONFIG_AuD_SOC1_CP1500_PROTO_VERSION == 3)       //neue HW - Proto Model 3

// LED1 GPIO83 red GPIO_OUT_2 SF_LED
#define BUSERR_LED_SOC			P19 
// LED2 GPIO80 gelb GPIO_OUT_2 MAINT_LED
#define SYNC_LED_SOC			P16

#else

// LED1 GPIO82 red GPIO_OUT_2
#define BUSERR_LED_SOC			P18
// LED7 GPIO88 red GPIO_OUT_2
#define SYNC_LED_SOC			P24

#endif
#endif

/*----------------------------------------------------------------------*/

#define IO_WORD(start, offset)	(*(volatile unsigned long *)(((start)+(offset))))

/* Port	Data Direction register ------------- */
#define IO_GPIO_IOCTRL_0		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_0)
#define IO_GPIO_IOCTRL_1		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_1)
#define IO_GPIO_IOCTRL_2		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_2)
#define IO_GPIO_IOCTRL_3		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_3)
#define IO_GPIO_IOCTRL_4		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_4)
#define IO_GPIO_IOCTRL_5		IO_WORD(SOC_GPIO_START, GPIO_IOCTRL_5)

/* Port	Data Out register ------------------- */
#define IO_GPIO_OUT_0			IO_WORD(SOC_GPIO_START, GPIO_OUT_0)
#define IO_GPIO_OUT_1			IO_WORD(SOC_GPIO_START, GPIO_OUT_1)
#define IO_GPIO_OUT_2			IO_WORD(SOC_GPIO_START, GPIO_OUT_2)
#define IO_GPIO_OUT_3			IO_WORD(SOC_GPIO_START, GPIO_OUT_3)
#define IO_GPIO_OUT_4			IO_WORD(SOC_GPIO_START, GPIO_OUT_4)
#define IO_GPIO_OUT_5			IO_WORD(SOC_GPIO_START, GPIO_OUT_5)

/* bit selective output register ------------------- */
#define IO_GPIO_OUT_SET_0		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_0)
#define IO_GPIO_OUT_SET_1		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_1)
#define IO_GPIO_OUT_SET_2		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_2)
#define IO_GPIO_OUT_SET_3		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_3)
#define IO_GPIO_OUT_SET_4		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_4)
#define IO_GPIO_OUT_SET_5		IO_WORD(SOC_GPIO_START, GPIO_OUT_SET_5)

/* bit selective clear register ------------------- */
#define IO_GPIO_OUT_CLEAR_0		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_0)
#define IO_GPIO_OUT_CLEAR_1		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_1)
#define IO_GPIO_OUT_CLEAR_2		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_2)
#define IO_GPIO_OUT_CLEAR_3		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_3)
#define IO_GPIO_OUT_CLEAR_4		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_4)
#define IO_GPIO_OUT_CLEAR_5		IO_WORD(SOC_GPIO_START, GPIO_OUT_CLEAR_5)

/* Port	Data IN register -------------------- */
#define IO_GPIO_IN_0			IO_WORD(SOC_GPIO_START, GPIO_IN_0)
#define IO_GPIO_IN_1			IO_WORD(SOC_GPIO_START, GPIO_IN_1)
#define IO_GPIO_IN_2			IO_WORD(SOC_GPIO_START, GPIO_IN_2)
#define IO_GPIO_IN_3			IO_WORD(SOC_GPIO_START, GPIO_IN_3)
#define IO_GPIO_IN_4			IO_WORD(SOC_GPIO_START, GPIO_IN_4)
#define IO_GPIO_IN_5			IO_WORD(SOC_GPIO_START, GPIO_IN_5)

/* Port l-mode register --------------------- */
#define IO_GPIO_PORT_MODE_0_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_0_L)
#define IO_GPIO_PORT_MODE_1_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_1_L)
#define IO_GPIO_PORT_MODE_2_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_2_L)
#define IO_GPIO_PORT_MODE_3_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_3_L)
#define IO_GPIO_PORT_MODE_4_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_4_L)
#define IO_GPIO_PORT_MODE_5_L		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_5_L)

/* Port h-mode register --------------------- */
#define IO_GPIO_PORT_MODE_0_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_0_H)
#define IO_GPIO_PORT_MODE_1_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_1_H)
#define IO_GPIO_PORT_MODE_2_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_2_H)
#define IO_GPIO_PORT_MODE_3_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_3_H)
#define IO_GPIO_PORT_MODE_4_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_4_H)
#define IO_GPIO_PORT_MODE_5_H		IO_WORD(SOC_GPIO_START, GPIO_PORT_MODE_5_H)

#endif
 
