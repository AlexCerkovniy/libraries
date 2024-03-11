#include <stdint.h>
#include <stdbool.h>

typedef enum{
	BVNA_CMD_PING = 0U,
	BVNA_CMD_REGISTRATION,
	BVNA_CMD_STATUS,
	BVNA_CMD_EXECUTE
}bvna_cmd_t;

#pragma pack(push, 1)

typedef struct{
	uint8_t repeater_id;		/* [0] - reserved for hub, [1...254] - used for repeaters, [255] - broadcast for all repeaters*/
	uint8_t device_id;			/* [0] - reserved for repeater,  [1...254] - used for devices, [255] - broadcast for all devices */
	uint8_t packet_number;
} bvna_radio_header_t;

typedef struct{
	uint8_t command;
	uint8_t device_type;
} bvna_radio_reg_request_t;

typedef struct{
	uint8_t command;
	uint8_t id;
	uint8_t hub_id[16];
} bvna_radio_reg_data_t;

typedef struct{
	uint8_t command;
	int8_t temperature_c;
	uint8_t battery_soc;
} bvna_radio_status_data_t;

typedef struct{
	uint8_t command;
} bvna_radio_status_req_t;

#pragma pack(pop)
