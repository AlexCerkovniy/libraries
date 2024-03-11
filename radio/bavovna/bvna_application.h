#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "bvna_configuration.h"

/* Default log print override, if not declared */
#ifndef BVNA_LOG
	#define BVNA_LOG(...)							do{}while(0)
#endif

#define BVNA_MAX_ANSWER_TIMEOUT_MS					(1000)

/* Repeaters configuration */
#define BVNA_REPEATER_COUNT							(1)
#define BVNA_REPEATER_PING_LOSTS_FOR_OFFLINE		(3)

/* Endpoints configuration */
#define BVNA_DEVICE_COUNT							(8)
#define BVNA_DEVICE_PING_LOSTS_FOR_OFFLINE			(3)

/* Define device types (SLAVES ONLY) */
#define BVNA_DEV_TYPE_REPEATER						(0)
#define BVNA_DEV_TYPE_DETONATOR						(1)

/* Define registration mode constants */
#define BVNA_REGISTRATION_TIMEOUT_MS				(30000)
#define BVNA_REGISTRATION_SEND_REG_DATA_RETRIES		(3)

#define BVNA_REGISTRATION_WAIT_REG_DATA_TIMEOUT_MS	(1500)
#define BVNA_REGISTRATION_SEND_REG_REQ_RETRIES		(3)

typedef enum{
	BVNA_STATE_IDLE = 0U,
	BVNA_STATE_UNREGISTERED,
	BVNA_STATE_REGISTRATION,
	BVNA_STATE_POLLING,
}bvna_state_t;

typedef enum{
	BVNA_SUBSTATE_SEND_STATUS = 0U,
	BVNA_SUBSTATE_REQ_STATUS,
	BVNA_SUBSTATE_WAIT_STATUS,
	BVNA_SUBSTATE_WAIT_COMMAND,
	BVNA_SUBSTATE_SEND_REG_REQUEST,
	BVNA_SUBSTATE_WAIT_REG_REQUEST,
	BVNA_SUBSTATE_SEND_REG_DATA,
	BVNA_SUBSTATE_WAIT_REG_DATA,
	BVNA_SUBSTATE_SEND_PACKET,
}bvna_substate_t;

typedef struct{
	uint16_t dev_type;
	bool selected;
	bool available;
	bool offline;
	uint16_t battery_soc;
	int8_t temperature_c;
	int8_t rssi;

	/* Private data */
	uint16_t poll_timer;
} bvna_device_t;

/* Device data, saved in NVRAM. All registered devices is saved in this way */

typedef struct{
#ifdef BVNA_DEVICE_HUB
	uint16_t dev_type;
	uint16_t crc16;
#else
	uint8_t id;
	uint8_t hub_id[16];
	uint16_t crc16;
#endif
} bvna_nvram_device_data_t;

typedef struct{

#ifdef BVNA_DEVICE_HUB
	uint8_t hub_id[16];		/* Unique hub id. Used for radio packets encryption as AES128 key */
	uint16_t hub_id_crc16; 	/* Unique hub id CRC16 */
	bvna_nvram_device_data_t repeater[BVNA_REPEATER_COUNT];
	bvna_nvram_device_data_t device[BVNA_DEVICE_COUNT];
#else
	bvna_nvram_device_data_t device;
#endif
} bvna_nvram_data_t;

void bvna_init(void);
void bvna_main(void);
void bvna_tick(uint32_t period);

/* Registration process start routine */
#if defined(BVNA_DEVICE_HUB)
	void bvna_start_registration(uint8_t device_id, uint8_t device_type);
	bvna_device_t *bvna_get_device_status(uint8_t device_id);
	bvna_device_t *bvna_get_repeater_status(uint8_t repeater_id);
#elif defined(BVNA_DEVICE_SLAVE)
	void bvna_start_registration(void);
#endif


