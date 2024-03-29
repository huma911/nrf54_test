/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *const uart_dev_135 = DEVICE_DT_GET(DT_NODELABEL(uart135));

//static const struct device *const i2c_dev_130 = DEVICE_DT_GET(DT_NODELABEL(i2c130));
#define I2C_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec i2c_dev_130 = I2C_DT_SPEC_GET(I2C_NODE);

#define SPI_NODE DT_NODELABEL(mydevice)
#define SPI_SCREEN_NODE DT_NODELABEL(myscreen)

#define SPI_OP SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE

static const struct spi_dt_spec spi_dev_131 = SPI_DT_SPEC_GET(SPI_NODE, SPI_OP, 0);
static const struct spi_dt_spec spi_dev_120 = SPI_DT_SPEC_GET(SPI_SCREEN_NODE, SPI_OP, 0);

static struct k_mutex mydevice_oper_mutex;
static struct k_mutex myscreen_oper_mutex;

static int spi_mutex_lock(void)
{
	int ret;

	ret = k_mutex_lock(&mydevice_oper_mutex, K_FOREVER);
	if (ret) {
		//LOG_ERR("Failed to lock mutex: %d", ret);
		return ret;
	}

	/* If operation mode set to HOLD or the SPI_LOCK_ON is set when
	 * taking the mutex something is wrong
	 */
	if ((spi_dev_131.config.operation & SPI_HOLD_ON_CS) || (spi_dev_131.config.operation & SPI_LOCK_ON)) {
		//LOG_ERR("SPI_HOLD_ON_CS and SPI_LOCK_ON must be freed before releasing mutex");
		return -EPERM;
	}

	return 0;
}

/* Unlocks mutex and CS pin */
static int spi_mutex_unlock(void)
{
	int ret;
	/* If operation mode still set to HOLD or
	 * the SPI_LOCK_ON is still set when releasing the mutex
	 * something is wrong
	 */
	if ((spi_dev_131.config.operation & SPI_HOLD_ON_CS) || (spi_dev_131.config.operation & SPI_LOCK_ON)) {
		//LOG_ERR("SPI_HOLD_ON_CS and SPI_LOCK_ON must be freed before releasing mutex");
		return -EPERM;
	}

	ret = k_mutex_unlock(&mydevice_oper_mutex);
	if (ret) {
		//LOG_ERR("Failed to unlock mutex: %d", ret);
		return ret;
	}

	return 0;
}

static int spi_120_mutex_lock(void)
{
	int ret;

	ret = k_mutex_lock(&myscreen_oper_mutex, K_FOREVER);
	if (ret) {
		//LOG_ERR("Failed to lock mutex: %d", ret);
		return ret;
	}

	/* If operation mode set to HOLD or the SPI_LOCK_ON is set when
	 * taking the mutex something is wrong
	 */
	if ((spi_dev_120.config.operation & SPI_HOLD_ON_CS) || (spi_dev_120.config.operation & SPI_LOCK_ON)) {
		//LOG_ERR("SPI_HOLD_ON_CS and SPI_LOCK_ON must be freed before releasing mutex");
		return -EPERM;
	}

	return 0;
}

/* Unlocks mutex and CS pin */
static int spi_120_mutex_unlock(void)
{
	int ret;
	/* If operation mode still set to HOLD or
	 * the SPI_LOCK_ON is still set when releasing the mutex
	 * something is wrong
	 */
	if ((spi_dev_120.config.operation & SPI_HOLD_ON_CS) || (spi_dev_120.config.operation & SPI_LOCK_ON)) {
		//LOG_ERR("SPI_HOLD_ON_CS and SPI_LOCK_ON must be freed before releasing mutex");
		return -EPERM;
	}

	ret = k_mutex_unlock(&myscreen_oper_mutex);
	if (ret) {
		//LOG_ERR("Failed to unlock mutex: %d", ret);
		return ret;
	}

	return 0;
}

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev_135)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev_135)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev_135, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

void print_uart_135(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev_135, buf[i]);
	}
}

int main(void)
{
	char tx_buf[MSG_SIZE];

	printk("Welcome to the test system!");

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	if (!device_is_ready(uart_dev_135)) {
		printk("UART device not found!");
		return 0;
	}
	else {
		print_uart("UART 135 device found!\r\n");
	}

	//static const struct i2c_dt_spec i2c_dev_130 = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(i2c_dev_130.bus)) {
		print_uart("I2C 130 device not found!");
		//return 0;
	}
	else {
		print_uart("I2C 130 device found!\r\n");
	}


	if (!device_is_ready(spi_dev_131.bus)) {
		print_uart("SPI 131 device not found!");
		//return 0;
	}
	else {
		print_uart("SPI 131 device found!\r\n");
	}

	if (!device_is_ready(spi_dev_120.bus)) {
		print_uart("SPI 120 device not found!");
		//return 0;
	}
	else {
		print_uart("SPI 120 device found!\r\n");
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev_135, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev_135);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");
	print_uart_135("Hello! I'm your nRF54 UART test device.\r\n");
	print_uart_135("Input something and press enter:\r\n");

	uint8_t config[2] = {0x03,0x8C};

	//ret = i2c_write_dt(&i2c_dev_130, config, sizeof(config));
	//if(ret != 0){
	//	print_uart("Failed to write to I2C device. \n");
	//	//return -1;
	//}

	uint8_t write_buffer[4] = {0x03,0x8C,0x03,0x8C};
	uint8_t pad_buffer[4] = { 0 };
	uint8_t data_buffer[4] = { 0 };
	struct spi_buf_set spi_tx;
	struct spi_buf spi_tx_buf[] = { { .buf = write_buffer, .len = 4 },
				    			{ .buf = pad_buffer, .len = 4 },
				    			{ .buf = data_buffer, .len = 4 } };

	spi_tx.buffers = spi_tx_buf;
	spi_tx.count = ARRAY_SIZE(spi_tx_buf);

	//ret = spi_mutex_lock();
	//if (ret) {
	//	print_uart("Failed to write to SPI 130 device.BSP_STATUS_FAIL. \n");
	//}
//
	//ret = spi_write_dt(&spi_dev_131, &spi_tx);
	//if(ret != 0){
	//	print_uart("Failed to write to SPI 130 device. \n");
	//	//return -1;
	//}
//
	//ret = spi_mutex_unlock();
	//if (ret) {
	//	print_uart("Failed to write to SPI 130 device.BSP_STATUS_FAIL. \n");
	//}

	ret = spi_120_mutex_lock();
	if (ret) {
		print_uart("Failed to write to SPI 120 device.BSP_STATUS_FAIL. \n");
	}

	ret = spi_write_dt(&spi_dev_120, &spi_tx);
	if(ret != 0){
		print_uart("Failed to write to SPI 120 device. \n");
		//return -1;
	}

	ret = spi_120_mutex_unlock();
	if (ret) {
		print_uart("Failed to write to SPI 120 device.BSP_STATUS_FAIL. \n");
	}

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		print_uart_135("Echo: ");
		print_uart_135(tx_buf);
		print_uart_135("\r\n");
	}
	return 0;
}
