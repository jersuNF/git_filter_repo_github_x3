/*
 * Copyright (c) 2022 Nofence AS
 */

#include "passthrough.h"

#include <zephyr.h>
#include <sys/ring_buffer.h>
#include <drivers/uart.h>

static atomic_t passthrough_tx_in_progress = ATOMIC_INIT(0);
static uint8_t passthrough_tx_buffer[CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE];
static uint32_t passthrough_tx_count = 0;
static uint32_t passthrough_tx_sent = 0;

/* Passthrough variables for UART */
static atomic_t passthrough_enabled = ATOMIC_INIT(0);
static enum diagnostics_interface passthrough_interface = DIAGNOSTICS_NONE;
static struct ring_buf passthrough_ring_buf;
static uint8_t* passthrough_buffer = NULL;

static const struct device *passthrough_device = NULL;

static void passthrough_flush(const struct device *uart_dev);
static void passthrough_uart_isr(const struct device *uart_dev,
				 void *user_data);
static void passthrough_uart_handle_tx(const struct device *uart_dev);
static void passthrough_uart_handle_rx(const struct device *uart_dev);

int passthrough_init(void) 
{
	return 0;
}

int passthrough_enable(enum diagnostics_interface intf, 
			const struct device *dev)
{
	passthrough_interface = intf;
	passthrough_device = dev;

	/* Make sure interrupts are disabled */
	uart_irq_rx_disable(passthrough_device);
	uart_irq_tx_disable(passthrough_device);
	
	/* Allocate buffer if not already done */
	if (passthrough_buffer == NULL)
	{
		passthrough_buffer = 
			k_malloc(CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);

		if (passthrough_buffer == NULL) {
			return -ENOBUFS;
		}
		ring_buf_init(&passthrough_ring_buf, 
			      CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE, 
			      passthrough_buffer);
	}
	
	if (passthrough_device == NULL) {
		return -EIO;
	}
	if (!device_is_ready(passthrough_device)) {
		return -EIO;
	}
	
	uart_irq_rx_disable(passthrough_device);
	uart_irq_tx_disable(passthrough_device);

	passthrough_flush(passthrough_device);

	uart_irq_callback_set(passthrough_device, passthrough_uart_isr);
	uart_irq_rx_enable(passthrough_device);

	atomic_set_bit(&passthrough_enabled, 0);

	return 0;
}

int passthrough_disable(void)
{
	atomic_clear_bit(&passthrough_enabled, 0);

	return 0;
}

void passthrough_get_enabled_interface(enum diagnostics_interface *intf)
{
	*intf = DIAGNOSTICS_NONE;

	if (atomic_test_bit(&passthrough_enabled, 0)) {
		*intf = passthrough_interface;
	}
}

uint32_t passthrough_claim_read_data(uint8_t** data)
{
	if (ring_buf_is_empty(&passthrough_ring_buf)) {
		/* No new data */
		return 0;
	}

	uint32_t size = ring_buf_get_claim(
				&passthrough_ring_buf, 
				data, 
				CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);

	return size;
}

int passthrough_finish_read_data(uint32_t size)
{
	return ring_buf_get_finish(&passthrough_ring_buf, size);
}

uint32_t passthrough_write_data(uint8_t* data, uint32_t size)
{
	uint32_t bytes_parsed = 0;
	if (!atomic_test_bit(&passthrough_tx_in_progress, 0)) {
		bytes_parsed = size;
		if (bytes_parsed > CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE) {
			bytes_parsed = CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE;
		}
		memcpy(passthrough_tx_buffer, data, bytes_parsed);
		passthrough_tx_count = bytes_parsed;
		passthrough_tx_sent = 0;
		
		atomic_set_bit(&passthrough_tx_in_progress, 0);
		uart_irq_tx_enable(passthrough_device);
	}

	return bytes_parsed;
}

static void passthrough_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

static void passthrough_uart_isr(const struct device *uart_dev,
				 void *user_data)
{
	while (uart_irq_update(uart_dev) &&
	       uart_irq_is_pending(uart_dev)) {
		
		if (uart_irq_tx_ready(uart_dev)) {
			passthrough_uart_handle_tx(uart_dev);
		}

		if (uart_irq_rx_ready(uart_dev)) {
			passthrough_uart_handle_rx(uart_dev);
		}
	}
}

static void passthrough_uart_handle_tx(const struct device *uart_dev)
{
	int bytes = 
		uart_fifo_fill(uart_dev, 
			       &passthrough_tx_buffer[passthrough_tx_sent], 
			       passthrough_tx_count - passthrough_tx_sent);
	passthrough_tx_sent += bytes;
	
	if (passthrough_tx_sent >= passthrough_tx_count) {
		atomic_clear_bit(&passthrough_tx_in_progress, 0);
		passthrough_tx_count = 0;
		passthrough_tx_sent = 0;
		uart_irq_tx_disable(uart_dev);
	}
}

static void passthrough_uart_handle_rx(const struct device *uart_dev)
{	
	uint8_t* data;
	uint32_t partial_size = 
		ring_buf_put_claim(&passthrough_ring_buf, 
				   &data, 
				   CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);

	if (partial_size == 0) {
		passthrough_flush(uart_dev);
		return;
	}

	uint32_t received = uart_fifo_read(uart_dev, 
					   data, 
					   partial_size);
	
	ring_buf_put_finish(&passthrough_ring_buf, received);
}