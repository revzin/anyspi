/* --------------------------------------------------------------------
MIT License

Copyright (c) 2016 Grigory Revzin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------ */

#ifndef __ANYSPI_H__
#define __ANYSPI_H__

/* enables extra internal checks and asserts
 * undefine that when you are positive everything is working fine */
#define ANYSPI_MORE_SECURE

#include <stddef.h> /* size_t */
#include <stdint.h>

#ifndef ANYSPI_MAX_INSTANCES
#define ANYSPI_MAX_INSTANCES (3)
#endif

typedef void* ANYSPI_InstanceHandle;

typedef enum {
	ANYSPI_OK = 0,
	ANYSPI_BAD_SETTINGS,
	ANYSPI_OUT_OF_MEMORY,
	ANYSPI_TX_EMPTY,
	ANYSPI_RX_EMPTY,
	ANYSPI_MAX_INSTANCE_NUMBER,
	ANYSPI_BAD_HANDLE,
	ANYSPI_INIT_NOT_DONE,
	ANYSPI_ALREADY_INITED,
	ANYSPI_TX_IN_PROGRESS,
	ANYSPI_IDLE
} ANYSPI_rc_t;

typedef enum {
	BITVAL_ZERO = 0,
	BITVAL_ONE = 1
} ANYSPI_bit_t;

typedef enum {
	ANYSPI_ENDIAN_BIG,
	ANYSPI_ENDIAN_LITTLE
} ANYSPI_endian_t;

typedef enum {
	ANYSPI_MSB_FIRST,
	ANYSPI_LSB_FIRST
} ANYSPI_bit_order_t;

typedef enum {
	ANYSPI_CPOL_0,
	ANYSPI_CPOL_1
} ANYSPI_CPOL;

typedef enum {
	ANYSPI_CPHA_0,
	ANYSPI_CPHA_1
} ANYSPI_CPHA;

typedef enum {
	ANYSPI_CS_NORMAL,
	ANYSPI_CS_SUBMIT_EVERY_FRAME
} ANYSPI_cs_submit_t;

typedef enum {
	ANYSPI_TXRX_NONMAL,
	ANYSPI_TXRX_IGNORE_1ST_BIT
} ANYSPI_txrx_ignore_1st_t;


typedef struct {
	void *port_address;
	int number;
	int disabled;
} ANYSPI_pin;


typedef ANYSPI_bit_t 	(*ANYSPI_fPinRead)
							(ANYSPI_InstanceHandle hInst, ANYSPI_pin *pin);
typedef void 			(*ANYSPI_fPinWrite)
							(ANYSPI_InstanceHandle hInst, ANYSPI_pin *pin, ANYSPI_bit_t value);

/* example gpio_read and gpio_write implementations for STM32 */
/*
ANYSPI_bit_t gpio_read(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *) pin->port_address;
	return (ANYSPI_bit_t) (((gpio->ODR & (1 << pin->number)) >> pin->number) > 0);
}

void gpio_write(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin, ANYSPI_bit_t value)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)pin->port_address;
	if (value == BITVAL_ONE)
		gpio->ODR = gpio->ODR | (1 << pin->number);
	else
		gpio->ODR = gpio->ODR & ~(1 << pin->number);
}
--------------------------------------------------------------*/

typedef void 			(*ANYSPI_fTxCompleteCallback) (ANYSPI_InstanceHandle hInst);

typedef struct {
	size_t bits_per_frame;
	size_t bytes_per_elem;
	size_t fifo_size;
	ANYSPI_endian_t endian;
	ANYSPI_bit_order_t bit_order;
	ANYSPI_fTxCompleteCallback tx_complete_callback;
	ANYSPI_fPinRead pin_read;
	ANYSPI_fPinWrite pin_write;
	ANYSPI_pin miso, mosi, cs, ck;
	ANYSPI_CPHA cpha;
	ANYSPI_CPOL cpol;
	ANYSPI_cs_submit_t submit;
	ANYSPI_txrx_ignore_1st_t ignore_1st;
} ANYSPI_settings;

/* inserts default values into the ANYSPI_settings struct */
void 		ANYSPI_DefaultSettings(ANYSPI_settings *settings);

/* writes an instance handle to hInst_out, if instance is successfully allocated
 * example:
 * 	ANYSPI_InstanceHandle h;
 *	if (ANYSPI_GetInstance(&h) != ANYSPI_OK)
 *		return 0; */
ANYSPI_rc_t ANYSPI_GetInstance(ANYSPI_InstanceHandle *hInst_out);

/* inits an instance with settings */
ANYSPI_rc_t ANYSPI_Init(ANYSPI_InstanceHandle hInst, ANYSPI_settings *settings);

/* pushes data to be transmitted */
ANYSPI_rc_t ANYSPI_PushTx(ANYSPI_InstanceHandle hInst, uint8_t *data_in);

/* pops received data */
ANYSPI_rc_t ANYSPI_PopRx(ANYSPI_InstanceHandle hInst, uint8_t *data_out);

/* starts transmission */
ANYSPI_rc_t ANYSPI_Start(ANYSPI_InstanceHandle hInst);

/* checks if line is idle */
ANYSPI_rc_t ANYSPI_IsIdle(ANYSPI_InstanceHandle hInst);

/* updates settings */
ANYSPI_rc_t ANYSPI_UpdateSettings(ANYSPI_InstanceHandle hInst, ANYSPI_settings *settings);

/* handles the transmissions -- generally to be called from a timer IRQ handler */
ANYSPI_rc_t ANYSPI_IRQHandler(void);

/* destroys the instance */
ANYSPI_rc_t ANYSPI_Cleanup(ANYSPI_InstanceHandle hInst);


#endif /* __ANYSPI_H__ */
