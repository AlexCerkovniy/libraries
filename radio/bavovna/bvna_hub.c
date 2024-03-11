#include "bvna_application.h"
#include "bvna_packet.h"
#include "bvna_callback.h"
#include "bvna_error.h"

#include "sx127x_lora.h"
#include "sx127x_driver.h"
#include "main.h"
#include "flash.h"

#ifdef BVNA_DEVICE_HUB

/* Radio Object */
sx127x_lora_t ra01 = {
	/* Set-up driver */
	.driver.init 				= sx127x_init,
	.driver.reset 				= sx127x_reset,
	.driver.read 				= sx127x_read,
	.driver.write 				= sx127x_write,
	.driver.delay_ms 			= sx127x_delay_ms,

	/* Set parameters */
	.frequency             		= BVNA_BASE_FREQ_HZ,
	.spreding_factor        	= RFLR_MODEMCONFIG2_SF_7,
	.bandwidth			   		= RFLR_MODEMCONFIG1_BW_20_83_KHZ,
	.coding_rate               	= RFLR_MODEMCONFIG1_CODINGRATE_4_6,
	.rx_gain					= 0, /* AGC selected */
	.power				   		= 17,
	.power_output				= RFLR_PACONFIG_PASELECT_PABOOST, /* RA-01 have output on PABOOST pin */
	.over_current_protection 	= 0,
	.preamble_length			= 10,
	.syncword					= 0x12,
	.crc						= true
};

/* Bavovna data in NVRAM (and in RAM copy of it) */
bvna_nvram_data_t bvna_nvram __attribute__((section(".device_data")))  __attribute__((aligned(4)));
bvna_nvram_data_t bvna_nvram_ram;

/* Bavovna Container */
struct {
	bool radio_ready;
	bvna_state_t state;
	bvna_substate_t substate;
	uint32_t wait_timer;
	uint32_t timeout_timer;
	uint32_t retries;

	/* Radio Buffer */
	uint8_t rx_buffer[128];
	uint16_t rx_count;
	uint8_t tx_buffer[128];
	uint16_t tx_count;

	/* Packets containers */
	bvna_radio_header_t *rx_header;
	bvna_radio_header_t *tx_header;
	uint8_t packet_number;

	/* Registration data */
	uint8_t reg_device_type;
	uint8_t reg_device_id;
	uint8_t reg_repeater_id;

	/* Polling devices ID's */
	uint8_t poll_repeater_id;
	uint8_t poll_device_id;

	/* Devices data */
	bvna_device_t repeater[BVNA_REPEATER_COUNT];
	bvna_device_t device[BVNA_DEVICE_COUNT];
} bvna;

/* Private functions prototypes ----------------------------------------------------- */
static void bvna_registration_process(void);
static void bvna_polling_process(void);
static void bvna_make_reg_data_packet(uint8_t device_id, uint8_t device_type);
static void bvna_make_status_request(uint8_t repeater_id, uint8_t device_id);
static uint16_t crc16(uint8_t *pcBlock, uint16_t len);

void bvna_init(void){
	uint16_t crc;

	if(sx127x_lora_init(&ra01) != LORA_OK){
		BVNA_LOG("Radio init error!\r\n");
		bvna_error_callback(BVNA_ERROR_RADIO_INIT);
		return;
	}
	sx127x_lora_set_low_data_rate_optimization(&ra01, true);
	sx127x_lora_start_rx(&ra01);

	/* Recover registered devices info from NVRAM  */
	memcpy(&bvna_nvram_ram, &bvna_nvram, sizeof(bvna_nvram_data_t));

	crc = crc16((uint8_t *)&bvna_nvram_ram.hub_id, 16);
	if(crc == bvna_nvram_ram.hub_id_crc16){
		BVNA_LOG("Hub ID recovered\r\n");
	} else {
		BVNA_LOG("Hub ID not available\r\n");
	}

	uint8_t id;
	for(id = 0; id < BVNA_REPEATER_COUNT; id++){
		crc = crc16((uint8_t *)&bvna_nvram_ram.repeater[id], sizeof(bvna_nvram_device_data_t) - 2);
		if(crc == bvna_nvram_ram.repeater[id].crc16){
			bvna.repeater[id].dev_type = bvna_nvram_ram.repeater[id].dev_type;
			bvna.repeater[id].available = true;
			bvna.repeater[id].offline = true;
			bvna.repeater[id].poll_timer = 0;
			BVNA_LOG("Repeater #%u reg. data recovered\r\n", id + 1);
		} else {
			BVNA_LOG("Repeater #%u reg. data empty\r\n", id + 1);
		}
	}

	for(id= 0; id < BVNA_DEVICE_COUNT; id++){
		crc = crc16((uint8_t *)&bvna_nvram_ram.device[id], sizeof(bvna_nvram_device_data_t) - 2);
		if(crc == bvna_nvram_ram.device[id].crc16){
			bvna.device[id].dev_type = bvna_nvram_ram.device[id].dev_type;
			bvna.device[id].available = true;
			bvna.device[id].offline = true;
			bvna.device[id].selected = false;
			bvna.device[id].poll_timer = 0;
			BVNA_LOG("Device #%u reg. data recovered\r\n", id + 1);
		} else {
			BVNA_LOG("Device #%u reg. data empty\r\n", id + 1);
		}
	}

	/* Initialize bavovna states */
	bvna.wait_timer = 0;
	bvna.timeout_timer = 0;
	bvna.state = BVNA_STATE_POLLING;
	bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
	bvna.poll_repeater_id = 1;
	bvna.poll_device_id = 1;
	bvna.packet_number = 0;
	bvna.retries = 0;
	bvna.rx_header = (bvna_radio_header_t *)bvna.rx_buffer;
	bvna.tx_header = (bvna_radio_header_t *)bvna.tx_buffer;
	bvna.rx_count = 0;
	bvna.tx_count = 0;
	bvna.radio_ready = true;
	BVNA_LOG("Started! Polling mode...\r\n");
}

void bvna_main(void){
	if(bvna.wait_timer || !bvna.radio_ready){
		return;
	}

	switch(bvna.state){
		case BVNA_STATE_REGISTRATION:
			bvna_registration_process();
			break;

		case BVNA_STATE_POLLING:
			bvna_polling_process();
			break;

		default:
			break;
	}
}

void bvna_tick(uint32_t period){
	static uint8_t id;

	if(bvna.wait_timer >= period){
		bvna.wait_timer -= period;
	}
	else{
		bvna.wait_timer = 0;
	}

	/* Polling periods count */
	for(id = 0; id < BVNA_REPEATER_COUNT; id++){
		if(bvna.repeater[id].available == false){
			continue;
		}

		if(bvna.repeater[id].poll_timer >= period){
			bvna.repeater[id].poll_timer -= period;
		}
		else{
			bvna.repeater[id].poll_timer = 0;
		}
	}

	for(id = 0; id < BVNA_DEVICE_COUNT; id++){
		if(bvna.device[id].available == false){
			continue;
		}

		if(bvna.device[id].poll_timer >= period){
			bvna.device[id].poll_timer -= period;
		}
		else{
			bvna.device[id].poll_timer = 0;
		}
	}
}

void bvna_start_registration(uint8_t device_id, uint8_t device_type){
	bvna.wait_timer = 0;
	bvna.timeout_timer = 0;
	bvna.retries = 0;
	bvna.state = BVNA_STATE_REGISTRATION;
	bvna.substate = BVNA_SUBSTATE_WAIT_REG_REQUEST;

	/* Make registration data packet */
	bvna_make_reg_data_packet(device_id, device_type);
	BVNA_LOG("Registration started: REP_ID[0x%.2X] DEV_ID[0x%.2X] DEV_TYPE[0x%.2X]\r\n",
			bvna.reg_repeater_id, bvna.reg_device_id, bvna.reg_device_type);

	/* Start/wait registration request at BVNA_BASE_FREQ_HZ */
	sx127x_lora_start_rx(&ra01);
}

void bvna_stop_registration(void){
	bvna.state = BVNA_STATE_POLLING;
	bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
	bvna.wait_timer = BVNA_RADIO_POLLING_MS;
	bvna.timeout_timer = 0;
	bvna.retries = 0;
	BVNA_LOG("Registration stopped\r\n");
}

bvna_device_t *bvna_get_device_status(uint8_t device_id){
	if(device_id >= BVNA_DEVICE_COUNT){
		return NULL;
	}

	return &bvna.device[device_id];
}

bvna_device_t *bvna_get_repeater_status(uint8_t repeater_id){
	if(repeater_id >= BVNA_REPEATER_COUNT){
		return NULL;
	}

	return &bvna.repeater[repeater_id];
}

static void bvna_registration_process(void){
	static bvna_radio_reg_request_t *reg_request = (bvna_radio_reg_request_t *)(bvna.rx_buffer + sizeof(bvna_radio_header_t));
	static bvna_radio_status_data_t *status_data = (bvna_radio_status_data_t *)(bvna.rx_buffer + sizeof(bvna_radio_header_t));

	switch(bvna.substate){
		case BVNA_SUBSTATE_WAIT_REG_REQUEST: /* Wait registration request at BVNA_BASE_FREQ_HZ */
			bvna.rx_count = sx127x_lora_available(&ra01);
			if(bvna.rx_count){
				bvna.rx_count = sx127x_lora_receive(&ra01, bvna.rx_buffer, bvna.rx_count);
				BVNA_LOG("RX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.rx_header->repeater_id,
							bvna.rx_header->device_id, *(bvna.rx_buffer + sizeof(bvna_radio_header_t)));

				/* Check if it's valid registration header */
				if(bvna.rx_header->repeater_id == 0 && bvna.rx_header->device_id == 0 &&
						reg_request->command == BVNA_CMD_REGISTRATION &&
						reg_request->device_type == bvna.reg_device_type){
					bvna.retries = 0;
					bvna.timeout_timer = 0;
					bvna.substate = BVNA_SUBSTATE_SEND_REG_DATA;
					BVNA_LOG("Registration request received\r\n");
					break;
				}
			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;

			/* Check timeout */
			bvna.timeout_timer += BVNA_RADIO_POLLING_MS;
			if(bvna.timeout_timer > BVNA_REGISTRATION_TIMEOUT_MS){
				/* Registration request not received, return to polling state */
				bvna.state = BVNA_STATE_POLLING;
				bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
				bvna.wait_timer = BVNA_RADIO_POLLING_MS;
				BVNA_LOG("Registration timeout. Go to polling...\r\n");
				bvna_registration_timeout_callback();
			}
			break;

		case BVNA_SUBSTATE_SEND_REG_DATA:	/* Send registration data at BVNA_BASE_FREQ_HZ */
			if(sx127x_lora_transmit(&ra01, bvna.tx_buffer, bvna.tx_count, 1000) != LORA_OK){
				bvna_error_callback(BVNA_ERROR_TX_TIMEOUT);
				BVNA_LOG("Radio TX timeout\r\n");

				/* Try send again */
				bvna.wait_timer = BVNA_RADIO_POLLING_MS;

				/* Return to polling mode if retries limit reached */
				if(++bvna.retries >= BVNA_TX_ERRORS_FOR_CRITICAL_ERROR){
					bvna.state = BVNA_STATE_POLLING;
					bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
					bvna.wait_timer = BVNA_RADIO_POLLING_MS;
					bvna_error_callback(BVNA_ERROR_TX_ERROR);
					bvna_registration_timeout_callback();
					BVNA_LOG("Radio TX error due to retries limit\r\n");
				}
				break;
			}

			BVNA_LOG("TX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.tx_header->repeater_id,
						bvna.tx_header->device_id, *(bvna.tx_buffer + sizeof(bvna_radio_header_t)));

			/* Registration data is send, wait status as answer */
			BVNA_LOG("Registration data sended. Wait status...\r\n");
			bvna.substate = BVNA_SUBSTATE_WAIT_STATUS;
			bvna.timeout_timer = 0;
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			bvna.packet_number++;
			break;

		case BVNA_SUBSTATE_WAIT_STATUS:		/* Waiting for status answer from registered device at BVNA_BASE_FREQ_HZ */
			bvna.rx_count = sx127x_lora_available(&ra01);
			if(bvna.rx_count){
				bvna.rx_count = sx127x_lora_receive(&ra01, bvna.rx_buffer, bvna.rx_count);
				BVNA_LOG("RX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.rx_header->repeater_id,
							bvna.rx_header->device_id, *(bvna.rx_buffer + sizeof(bvna_radio_header_t)));

				/* Check if it's valid status packet from registered device*/
				if(bvna.rx_header->repeater_id == bvna.reg_repeater_id && bvna.rx_header->device_id == bvna.reg_device_id){
					if(status_data->command == BVNA_CMD_STATUS){
						bvna_device_t *device;
						bvna_nvram_device_data_t *device_nvram;
						if(bvna.reg_device_type == BVNA_DEV_TYPE_REPEATER){
							device = &bvna.repeater[bvna.reg_device_id];
							device_nvram = &bvna_nvram_ram.repeater[bvna.reg_device_id];
						}
						else{
							device = &bvna.device[bvna.reg_device_id];
							device_nvram = &bvna_nvram_ram.device[bvna.reg_device_id];
						}

						/* Update device status */
						device->dev_type = bvna.reg_device_type;
						device->available = true;
						device->offline = false;
						device->battery_soc = status_data->battery_soc;
						device->temperature_c = status_data->temperature_c;
						device->rssi = ra01.rssi;

						/* Save device data in NVRAM */
						device_nvram->dev_type = bvna.reg_device_type;
						device_nvram->crc16 = crc16((uint8_t *)device_nvram, sizeof(bvna_nvram_device_data_t) - 2);
						flash_write(&bvna_nvram, &bvna_nvram_ram, sizeof(bvna_nvram_data_t));
						BVNA_LOG("Device data saved in NVRAM\r\n");

						/* Registration successful. Go to polling mode */
						bvna.state = BVNA_STATE_POLLING;
						bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
						bvna.wait_timer = BVNA_DEVICE_POLLING_MS;
						bvna_registration_success_callback();
						BVNA_LOG("Device registered!\r\n");
						break;
					}
				}
			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;

			/* Check timeout */
			bvna.timeout_timer += BVNA_RADIO_POLLING_MS;
			if(bvna.timeout_timer > BVNA_MAX_ANSWER_TIMEOUT_MS){
				bvna.retries++;
				if(bvna.retries == BVNA_REGISTRATION_SEND_REG_DATA_RETRIES){
					BVNA_LOG("Registration timeout. Go to polling...\r\n");

					/* Registered device won't answer to registration packet */
					bvna.state = BVNA_STATE_POLLING;
					bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
					bvna.wait_timer = BVNA_DEVICE_POLLING_MS;
					bvna_registration_timeout_callback();
				}
				else{
					bvna.substate = BVNA_SUBSTATE_SEND_REG_DATA;
					BVNA_LOG("Status wait timeout. Try again...\r\n");
				}
			}
			break;

		default:
			return;
	}
}

static void bvna_polling_process(void){
	static bvna_radio_status_data_t *status_data = (bvna_radio_status_data_t *)(bvna.rx_buffer + sizeof(bvna_radio_header_t));
	static bvna_device_t *device;

	switch(bvna.substate){
		case BVNA_SUBSTATE_REQ_STATUS:
			/* Poll repeater */
			if(bvna.repeater[bvna.poll_repeater_id - 1].available){
				if(bvna.repeater[bvna.poll_repeater_id - 1].poll_timer == 0){
					/* Make status request packet */
					bvna_make_status_request(bvna.poll_repeater_id, 0);

					/* Send message request packet */
					if(sx127x_lora_transmit(&ra01, bvna.tx_buffer, bvna.tx_count, 1000) != LORA_OK){
						bvna_error_callback(BVNA_ERROR_TX_TIMEOUT);
						BVNA_LOG("Radio TX timeout\r\n");

						/* Try send again */
						bvna.wait_timer = BVNA_RADIO_POLLING_MS;
						break;
					}

					BVNA_LOG("TX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.tx_header->repeater_id,
									bvna.tx_header->device_id, *(bvna.tx_buffer + sizeof(bvna_radio_header_t)));

					/* Status request sended, go to status answer wait */
					BVNA_LOG("Repeater status request, retries - %u\r\n", bvna.retries);
					bvna.substate = BVNA_SUBSTATE_WAIT_STATUS;
					bvna.wait_timer = BVNA_RADIO_POLLING_MS;
					bvna.timeout_timer = 0;
					break;
				}
			}

//			/* Poll devices */
//			if(bvna.repeater[0].available && !bvna.repeater[0].offline){
//
//			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			break;

		case BVNA_SUBSTATE_WAIT_STATUS:
			bvna.rx_count = sx127x_lora_available(&ra01);
			if(bvna.rx_count){
				bvna.rx_count = sx127x_lora_receive(&ra01, bvna.rx_buffer, bvna.rx_count);
				BVNA_LOG("RX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.rx_header->repeater_id,
							bvna.rx_header->device_id, *(bvna.rx_buffer + sizeof(bvna_radio_header_t)));

				/* Check if it's valid header */
				if(bvna.rx_header->repeater_id == 0 && bvna.rx_header->device_id == 0){
					/* Skip packet */
					break;
				}

				if(bvna.rx_header->repeater_id > BVNA_REPEATER_COUNT || bvna.rx_header->device_id > BVNA_DEVICE_COUNT){
					/* Skip packet */
					break;
				}

				if(bvna.rx_header->device_id){
					device = &bvna.device[bvna.rx_header->device_id - 1];
				}
				else {
					device = &bvna.repeater[bvna.rx_header->repeater_id - 1];
				}

				/* Check command */
				if(status_data->command == BVNA_CMD_STATUS){
					device->battery_soc = status_data->battery_soc;
					device->temperature_c = status_data->temperature_c;
					device->poll_timer = BVNA_REPEATER_POLLING_MS;
					device->offline = false;

					/* Go to status request sub-state */
					bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
					bvna.wait_timer = BVNA_RADIO_POLLING_MS;
					bvna.timeout_timer = 0;
					bvna.wait_timer = 0;
					bvna.retries = 0;

					BVNA_LOG("Status request answer received\r\n");
				}

				break;
			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;

			/* Check timeout */
			bvna.timeout_timer += BVNA_RADIO_POLLING_MS;
			if(bvna.timeout_timer > BVNA_MAX_ANSWER_TIMEOUT_MS){
				if(++bvna.retries >= BVNA_REPEATER_PING_LOSTS_FOR_OFFLINE){
					bvna.repeater[bvna.poll_repeater_id - 1].offline = true;
					bvna.repeater[bvna.poll_repeater_id - 1].poll_timer = BVNA_REPEATER_POLLING_MS;
					bvna.retries = 0;
					BVNA_LOG("Status request answer timeout. Device offline\r\n");
				}

				bvna.substate = BVNA_SUBSTATE_REQ_STATUS;
				bvna.timeout_timer = 0;
				bvna.wait_timer = 0;
			}
			break;

		default:
			return;
	}
}

static void bvna_make_reg_data_packet(uint8_t device_id, uint8_t device_type){
	bvna_radio_header_t *header = (bvna_radio_header_t *)bvna.tx_buffer;
	bvna_radio_reg_data_t *command = (bvna_radio_reg_data_t *)(bvna.tx_buffer + sizeof(bvna_radio_header_t));

	/* Device ID 0 is reserved */
	if(device_id == 0){
		device_id = 1;
	}

	/* Save registered device info */
	bvna.reg_device_type = device_type;
	if(device_type == BVNA_DEV_TYPE_REPEATER){
		bvna.reg_repeater_id = device_id;
		bvna.reg_device_id = 0;
	}
	else{
		bvna.reg_repeater_id = 0;
		bvna.reg_device_id = device_id;
	}

	/* Make registration data packet */
	header->device_id = 0;
	header->repeater_id = 0;
	header->packet_number = bvna.packet_number++;
	command->command = BVNA_CMD_REGISTRATION;
	command->id = device_id;
	memcpy(command->hub_id, bvna_nvram.hub_id, 16);
	bvna.tx_count = sizeof(bvna_radio_header_t) + sizeof(bvna_radio_reg_data_t);
}

static void bvna_make_status_request(uint8_t repeater_id, uint8_t device_id){
	bvna_radio_header_t *header = (bvna_radio_header_t *)bvna.tx_buffer;
	bvna_radio_status_req_t *command = (bvna_radio_status_req_t *)(bvna.tx_buffer + sizeof(bvna_radio_header_t));

	/* Make status request packet */
	header->repeater_id = repeater_id;
	header->device_id = device_id;
	header->packet_number = bvna.packet_number++;
	command->command = BVNA_CMD_STATUS;
	bvna.tx_count = sizeof(bvna_radio_header_t) + sizeof(bvna_radio_status_req_t);
}

static uint16_t crc16(uint8_t *pcBlock, uint16_t len){
    uint16_t crc = 0xFFFF;
    uint8_t i;

    while (len--){
        crc ^= *pcBlock++ << 8;

        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }

    return crc;
}

#endif //#ifdef BVNA_DEVICE_HUB
