#ifndef H_MCAL
#define H_MCAL

#define IOPINS 10U

typedef struct {
	uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t can_dlc; /* frame payload length in byte (0 .. 8) */
	uint8_t data[8];
} can_frame;

typedef struct {
	uint8_t tv_sec;
	uint8_t tv_nsec;
} timespec;

void go_sleep(uint8_t seconds);
uint8_t set_pin_status(uint8_t p_status, uint8_t p_pin);
uint8_t read_pin_status(uint8_t *status, uint8_t pin);
void go_sleep(uint8_t seconds);
void show_error(char *msg);
void get_time(timespec *time);
void show_log(char *msg);
uint8_t can_send_vcan0(can_frame *frame);

#endif
