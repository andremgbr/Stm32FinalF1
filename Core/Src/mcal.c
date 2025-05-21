#include <unistd.h>
#include "ecu.h"
#include "main.h"
#include "mcal.h"
#include "pins.h"
#include <string.h>

char buffer[200] = { 0 };

int pins[10] = { 0 };

const uint16_t gpio_pin_map[] = {
		GPIO_PIN_1, // HAZARD_LIGHTS_PIN
		GPIO_PIN_2, // HAZARD_BUTTON_PIN
		GPIO_PIN_3, // REB_ACTIVATE_PIN
		GPIO_PIN_4, // REB_IPC_WARNING
		GPIO_PIN_5, // REB_DEACTIVATE
		GPIO_PIN_6, // ENGINE_REB_MODE
		GPIO_PIN_7, // REB_IPC_FAULT_PIN
		};

uint8_t dio_get_pin(uint8_t *status, uint8_t pin) {
	*status = pins[pin];
	return SUCCESS;
}

uint8_t dio_set_pin(uint8_t status, uint8_t pin) {
	pins[pin] = status;

	if (pin != 1 && pin != 2 && pin != 4) {

		sprintf(buffer, "tentativa de setar pino real: %d, com status %d \r\n",
				pin, status);
		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, gpio_pin_map[pin],
				status == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

	}
	return SUCCESS;
}

/**
 *  @brief Get the status of desired PIN number.
 *
 *  @param status Pointer to store the read status of the PIN (0 or 1).
 *  @param pin Pointer of PIN number to be read.
 *  @return SUCCESS(0), FAIL(1)
 *  @requir{SwHLR_F_13}
 */
uint8_t read_pin_status(uint8_t *status, uint8_t pin) {
	uint8_t return_status = ECU_FAIL;
	if (pin < IOPINS) {
		return_status = dio_get_pin(status, pin);
	}
	return return_status;
}

/**
 *  @brief Set the status of desired PIN number.
 *
 *  @param status Pointer to store the status of the PIN (0 or 1).
 *  @param pin Pointer of PIN number to be read.
 *  @return SUCCESS(0), FAIL(1)
 *  @requir{SwHLR_F_13}
 */
uint8_t set_pin_status(uint8_t p_status, uint8_t p_pin) {
	uint8_t return_status = ECU_FAIL;
	if (p_pin < IOPINS) {
		return_status = dio_set_pin(p_status, p_pin);
	}
	return return_status;
}

/**
 *  @brief Sleep thread POSIX.
 *
 *  @param seconds How many seconds to sleep.
 */
void go_sleep(uint8_t seconds) {
	osDelay(seconds * 1000);
}

void show_error(char *msg) {
	char buffer[300];
	sprintf(buffer, "%s \r\n", msg);
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
}

void get_time(timespec *time) {
	uint32_t tick_ms = HAL_GetTick();
	time->tv_sec = tick_ms / 1000;
	time->tv_nsec = (tick_ms % 1000) * 1000000;
}

void show_log(char *msg) {
	char buffer[300];
	sprintf(buffer, "%s \r\n", msg);
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
}

uint8_t can_send_vcan0(can_frame *frame) {

	uint32_t mailbox;

	CAN_TxHeaderTypeDef TxHeader;
	// Configure message
	TxHeader.StdId = frame->can_id;
	TxHeader.ExtId = 0x00;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;


	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, frame->data, &mailbox)
			!= HAL_OK) {
		char buffer[300];
		sprintf(buffer, "Error can_send_vcan0 \r\n");
		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	}

	return 0;
}
