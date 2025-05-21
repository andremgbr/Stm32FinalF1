#include <app.h>
#include <ecu.h>
#include <ecu_aux.h>
#include <mcal.h>
#include <pins.h>
#include "main.h"

uint8_t reb_con = 0;

/**
 * @brief function that makes hazard lights blink
 * @return SUCCESS(0); FAIL(1).
 */
uint8_t hazard_lights_blink(void) {
	uint8_t ret = ECU_SUCCESS;
	while (ret == ECU_SUCCESS) {
		uint8_t status = 0;
		if (get_hazard_button_status(&status) == ECU_FAIL) {
			show_error("get_hazard_button_status FAIL\n");
			ret = ECU_FAIL;
		} else if ((status == S_ON)) {
			if ((set_hazard_light(S_ON) == ECU_FAIL)) {
				show_error("set_hazard_light FAIL\n");
				ret = ECU_FAIL;
			};
			go_sleep(1);
			if ((ret == ECU_SUCCESS) && (set_hazard_light(S_OFF) == ECU_FAIL)) {
				show_error("set_hazard_light FAIL\n");
				ret = ECU_FAIL;
			};
			go_sleep(1);
		} else {
			go_sleep(2);
		}
		osDelay(0);
	}
	return ret;
}

/**
 * @brief Handle messages received from CAN BUS
 * @return SUCCESS(0); FAIL(1).
 * @requir{SwHLR_F_6}
 * @requir{SwHLR_F_10}
 * @requir{SwHLR_F_15}
 */
uint8_t monitor_read_can(void) {
	uint8_t status = ECU_SUCCESS;
	while (1) {
		can_frame frame = { .can_id = 29, .can_dlc = 8, .data = { 0xFF, 0xFF,
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };

		if (can_read_vcan0(&frame) != ECU_FAIL) {
			handle_frame_can(&frame);
		} else {
			show_error("Error monitor_read_can\n");
			go_sleep(2);
		}
	}
	return status;
}

uint8_t handle_frame_can(can_frame *frame) {
	uint8_t status = ECU_SUCCESS;
	// handle message from REB to ECU
	if (frame->can_id == REB_ECU_ID) {
		if (handle_ecu_can(frame->data) == ECU_FAIL) {
			show_error("Handle_ecu_can ERROR\n");
		}
	}

	// handle message from REB to IPC
	if (frame->can_id == REB_IPC_ID) {
		if (handle_ipc_can(frame->data) == ECU_FAIL) {
			show_error("handle_ipc_can ERROR\n");
		}
	}
	if ((frame->can_id == 0x015) && (frame->data[0] == 0x10)) {
		// Return as ok the communication between REB x AUX
		reb_con = 1;
	}
	return status;
}

/**
 * @brief Handle the Reb ON and Reb OFF Buttons from GUI
 * @return SUCCESS(0); FAIL(1).
 */
uint8_t monitor_tcu(void) {
	uint8_t ret = ECU_SUCCESS;
	while (ret == ECU_SUCCESS) {
		uint8_t status = 0;
		// get REB ON Button status
		if (get_tcu_start_reb(&status) == ECU_FAIL) {
			show_error("get_tcu_star_reb FAIL\n");
			ret = ECU_FAIL;
		}
		// Activate REB
		if ((ret == ECU_SUCCESS) && (status == S_ON)) {
			// Toggle OFF Button to not enter in this loop
			if (set_tcu_start_reb(S_OFF) == ECU_FAIL) {
				show_error("set_tcu_start_reb FAIL\n");
				ret = ECU_FAIL;
			}
			// Send frame CAN to REB UNIT to start REB
			if ((ret == ECU_SUCCESS)
					&& (tcu_can_send_reb(REB_START) == ECU_FAIL)) {
				show_error("tcu_can_send_reb FAIL\n");
				ret = ECU_FAIL;
			}
		}

		// get REB OFF Button status
		if ((ret == ECU_SUCCESS) && (get_tcu_cancel_reb(&status) == ECU_FAIL)) {
			show_error("get_tcu_cancel_reb FAIL\n");
			ret = ECU_FAIL;
		}
		// Deactivate REB
		if ((ret == ECU_SUCCESS) && (status == S_ON)) {
			// Toggle OFF Button to not enter in this loop
			if (set_tcu_cancel_reb(S_OFF) == ECU_FAIL) {
				show_error("set_tcu_cancel_reb FAIL\n");
				ret = ECU_FAIL;
			}
			// Send fram CAN to REB UNIT to cancel REB
			if ((ret == ECU_SUCCESS)
					&& (tcu_can_send_reb(REB_CANCEL) == ECU_FAIL)) {
				show_error("tcu_can_send_reb FAIL\n");
				ret = ECU_FAIL;
			}
		} else {
			go_sleep(2);
		}
		osDelay(0);
	}
	return ret;
}

/**
 * @brief Check communication CAN between REB e AUX modules sending a frame and receving a response.
 * @return SUCCESS(0); FAIL(1).
 * @requir{SwHLR_F_13}
 * @requir{SwHLR_F_15}
 */
uint8_t check_can_communication(void) {
	// Frames to check cSommunication CAN between REB e AUX modules
	can_frame test_frame = { 0 };
	test_frame.can_id = AUX_COM_ID;
	test_frame.can_dlc = 1;
	test_frame.data[0] = AUX_COM_SIG;

	while (1) {
		reb_con = 0;
		int cont_tries = 0;

		if (can_send_vcan0(&test_frame) == ECU_FAIL) {
			show_error(
					"check_can_communication: Error to send communication test\n");
		}
		uint8_t current_ipc_fault_pin_status = 0x00U;

		while (reb_con == 0x00U) {
			if (cont_tries >= 10) {
				// Try 10 times the communication test {SwHLR_F_15}
				if (read_pin_status(&current_ipc_fault_pin_status,
				REB_IPC_FAULT_PIN) == ECU_FAIL) {
					show_error("read_pin_status ERROR\n");
				}
				if (current_ipc_fault_pin_status != 0x01U) {
					if (set_pin_status(1, REB_IPC_FAULT_PIN) == ECU_FAIL) {
						show_error("set_pin_status ERROR\n");
					}
				}
				if (can_send_vcan0(&test_frame) == ECU_FAIL) {
					show_error(
							"check_can_communication: Timeout CAN communication\n");
				}
				cont_tries = 0;
			}
			// Verify communication each 11 seconds
			go_sleep(1);
			cont_tries++;
		}
		if (read_pin_status(&current_ipc_fault_pin_status,
		REB_IPC_FAULT_PIN) == ECU_FAIL) {
			show_error("read_pin_status ERROR\n");
		}
		if (current_ipc_fault_pin_status != 0x00U) {
			if (set_pin_status(0, REB_IPC_FAULT_PIN) == ECU_FAIL) {
				show_error("set_pin_status ERROR\n");
			}
		}
	}
	return ECU_SUCCESS;
}
