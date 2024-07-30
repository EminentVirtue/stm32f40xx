/*
 * lis2d.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Andrew Streng
 *
 */

#ifndef LIS2D_H_
#define LIS2D_H_

#include "stm32f4xx.h"
#include "spi.h"

/* LIS2D Register Offsets*/
#define RO_REGISTER 	0x1
#define RW_REGISTER		0x0

typedef struct {
	uint8_t OUT_T_L;
	uint8_t OUT_T_H;
	uint8_t WHO_AM_I;
	uint8_t CTRL1;
	uint8_t CTRL2;
	uint8_t CTRL3;
	uint8_t CTRL4_INT1_PAD_CTRL;
	uint8_t CTRL5_INT2_PAD_CTRL;
	uint8_t CTRL6;
	uint8_t STATUS;
	uint8_t OUT_X_L;
	uint8_t OUT_X_Y;
	uint8_t OUT_Y_L;
	uint8_t OUT_Y_H;
	uint8_t OUT_Z_L;
	uint8_t OUT_Z_H;
	uint8_t FIFO_CTRL;
	uint8_t FIFO_SAMPLES;
	uint8_t TAP_THS_X;
	uint8_t TAG_THS_Y;
	uint8_t TAG_THS_Z;
	uint8_t INT_DUR;
	uint8_t WAKE_UP_THS;
	uint8_t WAKE_UP_DUR;
	uint8_t FREE_FALL;
	uint8_t STATUS_DUP;
	uint8_t WAKE_UP_SRC;
	uint8_t TAP_SRC;
	uint8_t SIXD_SRC;
	uint8_t ALL_INT_SRC;
	uint8_t X_OFS_USR;
	uint8_t Y_OFS_USR;
	uint8_t Z_OFS_USR;
	uint8_t CTRL7;
}LIS2D_RegDef_t;

/*
#define OUT_T_L			0x0D
#define OUT_T_H			0x0E
#define WHO_AM_I		0x0F
#define CTRL1			0x20
#define CTRL2			0x21
#define CTRL3			0x22
#define CTRL4_INT1_PAD_CTRL	0x23
#define CTRL5_INT2_PAD_CTRL	0x24
#define CTRL6			0x25
#define STATUS			0x27
#define OUT_X_L			0x28
#define OUT_X_Y			0x29
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D
#define FIFO_CTRL		0x2E
#define FIFO_SAMPLES	0X2F
#define TAP_THS_X		0x30
#define TAG_THS_Y		0x31
#define TAG_THS_Z		0x32
#define INT_DUR			0x33
#define WAKE_UP_THS		0x34
#define WAKE_UP_DUR		0x35
#define FREE_FALL		0x36
#define STATUS_DUP		0x37
#define WAKE_UP_SRC		0x38
#define TAP_SRC			0x39
#define SIXD_SRC		0x3A
#define ALL_INT_SRC		0x3B
#define X_OFS_USR		0x3C
#define Y_OFS_USR		0x3D
#define Z_OFS_USR		0x3E
#define CTRL7			0x3F


#define LIS2D_SPI_4_WIRE	0x1
#define LIS2D_SPI_3_WIRE	0x2
*/

typedef struct {
	uint8_t wire_interface;			/* LIS2D supports 3 or 4 wire SPI communication */
}LIS2D_SPI_Handle_t;


typedef struct {

}LIS2D_I2C_Handle_t;

#endif /* LIS2D_H_ */
