#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/nvic.h>

#define USART_BUFFER_SIZE		1024

struct usart_buffer {
	int write;
	int read;
	int lost_bytes;
	int data[USART_BUFFER_SIZE];
};

volatile struct usart_buffer usart1_buffer;
volatile struct usart_buffer usart3_buffer;

void usart_buffer_init(volatile struct usart_buffer* b) 
{
	b->write = 1;
	b->read = 0;
	b->lost_bytes = 0;
}

/* Set up all the peripherals */
void setup(void)
{
	rcc_clock_setup_in_hsi_out_64mhz();

	rcc_peripheral_enable_clock(&RCC_APB1ENR, 
			RCC_APB1ENR_USART3EN);

	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
			RCC_APB2ENR_IOPAEN |
			RCC_APB2ENR_IOPBEN |
			RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_AFIOEN | 
			RCC_APB2ENR_USART1EN
			);

	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* GPIO pin for USART TX */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

	/* GPIO pin for USART RX */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO10);

	/* Setup USART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	USART1_CR1 |= USART_CR1_RXNEIE;

	usart_buffer_init(&usart1_buffer);
	usart_enable(USART1);

	/* RESET */
	gpio_set(GPIOB, GPIO7);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO7);

	/* TX */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
	/* RX */
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO11);

	/* CTS */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, GPIO13);
	/* RTS */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO14);

	AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;

	/* Setup USART parameters. */
	usart_set_baudrate(USART3, 38400);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_RTS_CTS);
	usart_set_mode(USART3, USART_MODE_TX_RX);

	USART3_CR1 |= USART_CR1_RXNEIE;

	usart_buffer_init(&usart3_buffer);
	usart_enable(USART3);
}

void usart_isr(uint32_t usart, volatile struct usart_buffer* b)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(usart) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(usart) & USART_SR_RXNE) != 0)) {

		char c = usart_recv(usart);

		if(b->write != b->read) {
			b->data[b->write] = c;

			b->write = (b->write + 1) % USART_BUFFER_SIZE;
		} else {
			b->lost_bytes++;
		}
	}
}

void usart1_isr(void)
{
	usart_isr(USART1, &usart1_buffer);
}

void usart3_isr(void)
{
	usart_isr(USART3, &usart3_buffer);
}

int usart_read(volatile struct usart_buffer* b, char* c)
{
	int next_read = (b->read + 1) % USART_BUFFER_SIZE;
	if(next_read == b->write) {
		return 0;
	} else {
		*c = b->data[next_read];
		b->read = next_read;
		return 1;
	}
}

int main(void)
{
	setup();

	printf("Boot\n");

	while(1) {
		char c;
		if(usart_read(&usart1_buffer, &c)) {
			usart_send_blocking(USART3, c);
		}
		if(usart_read(&usart3_buffer, &c)) {
			usart_send_blocking(USART1, c);
		}
	}
}
