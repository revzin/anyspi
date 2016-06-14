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

/* example implementation for STM32F4 MCUs */

#include "stm32f4xx.h"

#include "anyspi.h"

void GPIO_Prepare(void);

ANYSPI_bit_t gpio_read(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin);
void 		gpio_write(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin, ANYSPI_bit_t value);
void 		callback(ANYSPI_InstanceHandle inst);

int main(void)
{
	NVIC_EnableIRQ(SysTick_IRQn);

	SysTick_Config(0xFFFFF);

	GPIO_Prepare();

	ANYSPI_settings s;
	ANYSPI_DefaultSettings(&s);

	s.submit = ANYSPI_CS_SUBMIT_EVERY_FRAME;
	s.bit_order = ANYSPI_MSB_FIRST;

	s.bits_per_frame = 8;
	s.bytes_per_elem = 1;

	s.cpol = ANYSPI_CPOL_0;

	s.miso.number = 12;
	s.miso.port_address = GPIOA;

	s.mosi.number = 10;
	s.mosi.port_address = GPIOA;

	s.cs.number = 11;
	s.cs.port_address = GPIOA;

	s.ck.number = 8;
	s.ck.port_address = GPIOC;

	s.pin_read = gpio_read;
	s.pin_write = gpio_write;
	s.tx_complete_callback = callback;

	ANYSPI_InstanceHandle h;

	if (ANYSPI_GetInstance(&h) != ANYSPI_OK)
		while(1) __NOP();

	if (ANYSPI_Init(h, &s) != ANYSPI_OK)
		while(1) __NOP();

	while (1)
	{
		int data;
		data = 0xFF;
		ANYSPI_PushTx(h, (uint8_t *) &data);
		data = 0xFE;
		ANYSPI_PushTx(h, (uint8_t *) &data);
		data = 0xFD;
		ANYSPI_PushTx(h, (uint8_t *) &data);
		data = 0xFC;
		ANYSPI_PushTx(h, (uint8_t *) &data);
		data = 0xFB;
		ANYSPI_PushTx(h, (uint8_t *) &data);
		data = 0xFA;
		ANYSPI_PushTx(h, (uint8_t *) &data);

		ANYSPI_Start(h);
	}
}

void GPIO_Prepare(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0);
	SET_BIT(GPIOC->MODER, GPIO_MODER_MODER8_0);
}

ANYSPI_bit_t gpio_read(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *) pin->port_address;
	return (ANYSPI_bit_t) (((gpio->IDR & (1 << pin->number)) >> pin->number) > 0);
}

void gpio_write(ANYSPI_InstanceHandle inst, ANYSPI_pin *pin, ANYSPI_bit_t value)
{
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)pin->port_address;
	if (value == BITVAL_ONE)
		gpio->ODR = gpio->ODR | (1 << pin->number);
	else
		gpio->ODR = gpio->ODR & ~(1 << pin->number);
}

void callback(ANYSPI_InstanceHandle inst)
{
	static uint8_t data[10] = {0};
	int i = -1;

	while (ANYSPI_PopRx(inst, &data[++i]) != ANYSPI_RX_EMPTY) {;}

	i = 10;
}

void SysTick_Handler(void)
{
	ANYSPI_IRQHandler();
	SysTick_Config(0xFFFF);
}

