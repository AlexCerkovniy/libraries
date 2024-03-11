#include "bvna_application.h"
#include "bvna_packet.h"
#include "bvna_callback.h"
#include "bvna_error.h"

#include "sx127x_lora.h"
#include "sx127x_driver.h"
#include "main.h"
#include "flash.h"

#if defined(BVNA_DEVICE_SLAVE) && (BVNA_DEVICE_SLAVE_TYPE == BVNA_DEV_TYPE_REPEATER)

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
} bvna;

/* Private functions prototypes ----------------------------------------------------- */
static void bvna_registration_process(void);
static void bvna_polling_process(void);
static uint16_t crc16(uint8_t *pcBlock, uint16_t len);
static void bvna_make_reg_data_packet(void);
static void bvna_make_status_answer(void);

void bvna_init(void){
	if(sx127x_lora_init(&ra01) != LORA_OK){
		BVNA_LOG("Radio init error!\r\n");
		bvna_error_callback(BVNA_ERROR_RADIO_INIT);
		return;
	}
	sx127x_lora_set_low_data_rate_optimization(&ra01, true);

	/* Recover registered devices info from NVRAM  */
	memcpy(&bvna_nvram_ram, &bvna_nvram, sizeof(bvna_nvram_data_t));

	/* Restore device data */
	uint16_t crc = crc16((uint8_t *)&bvna_nvram_ram.device, sizeof(bvna_nvram_device_data_t) - 2);
	if(crc == bvna_nvram_ram.device.crc16){
		bvna.state = BVNA_STATE_POLLING;
		bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
		sx127x_lora_start_rx(&ra01);
		BVNA_LOG("Registration data recovered\r\n");
	} else {
		bvna.state = BVNA_STATE_UNREGISTERED;
		BVNA_LOG("Device unregistered\r\n");
	}

	/* Initialize bavovna states */
	bvna.wait_timer = 0;
	bvna.timeout_timer = 0;
	bvna.rx_count = 0;
	bvna.tx_count = 0;
	bvna.packet_number = 0;
	bvna.retries = 0;
	bvna.radio_ready = true;
	bvna.rx_header = (bvna_radio_header_t *)bvna.rx_buffer;
	bvna.tx_header = (bvna_radio_header_t *)bvna.tx_buffer;
	BVNA_LOG("Started...\r\n");
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
	if(bvna.wait_timer >= period){
		bvna.wait_timer -= period;
	}
	else{
		bvna.wait_timer = 0;
	}
}

void bvna_start_registration(void){
	bvna.state = BVNA_STATE_REGISTRATION;
	bvna.substate = BVNA_SUBSTATE_SEND_REG_REQUEST;
	bvna.wait_timer = 0;
	bvna.retries = 0;

	/* Make registration request packet */
	bvna_make_reg_data_packet();
	BVNA_LOG("Registration started\r\n");

	/* Start/wait registration request at BVNA_BASE_FREQ_HZ */
	sx127x_lora_set_op_mode(&ra01, RFLR_OPMODE_SYNTHESIZER_RX);
	sx127x_lora_set_rf_frequency(&ra01, BVNA_BASE_FREQ_HZ);
	sx127x_lora_start_rx(&ra01);
}

static void bvna_registration_process(void){
	static bvna_radio_reg_data_t *reg_data = (bvna_radio_reg_data_t *)(bvna.rx_buffer + sizeof(bvna_radio_header_t));

	switch(bvna.substate){
		case BVNA_SUBSTATE_SEND_REG_REQUEST: /* Send registration request at BVNA_BASE_FREQ_HZ */
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

			/* Registration request is send, wait data */
			bvna.substate = BVNA_SUBSTATE_WAIT_REG_DATA;
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			bvna.timeout_timer = 0;
			BVNA_LOG("Registration request sended, retrie %u\r\n", bvna.retries);
			break;

		case BVNA_SUBSTATE_WAIT_REG_DATA:
			bvna.rx_count = sx127x_lora_available(&ra01);
			if(bvna.rx_count){
				bvna.rx_count = sx127x_lora_receive(&ra01, bvna.rx_buffer, bvna.rx_count);
				BVNA_LOG("RX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.rx_header->repeater_id,
							bvna.rx_header->device_id, *(bvna.rx_buffer + sizeof(bvna_radio_header_t)));

				/* Check if it's valid registration header */
				if(bvna.rx_header->repeater_id == 0 && bvna.rx_header->device_id == 0 && reg_data->command == BVNA_CMD_REGISTRATION){
					/* Save registration data */
					bvna_nvram_ram.device.id = reg_data->id;
					memcpy(bvna_nvram_ram.device.hub_id, reg_data->hub_id, 16);
					bvna_nvram_ram.device.crc16 = crc16((uint8_t *)&bvna_nvram_ram.device, sizeof(bvna_nvram_device_data_t) - 2);
					flash_write(&bvna_nvram, &bvna_nvram_ram, sizeof(bvna_nvram_data_t));
					BVNA_LOG("Registration data saved!\r\n");

					/* Make status answer */
					bvna_make_status_answer();

					/* Send status */
					bvna.substate = BVNA_SUBSTATE_SEND_STATUS;
					bvna.wait_timer = BVNA_RADIO_POLLING_MS;
					bvna.timeout_timer = 0;
					bvna.retries = 0;
					break;
				}
			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;

			/* Check timeout */
			bvna.timeout_timer += BVNA_RADIO_POLLING_MS;
			if(bvna.timeout_timer > BVNA_REGISTRATION_WAIT_REG_DATA_TIMEOUT_MS){
				/* Registration request not received, re-send request (if retries limit not reached) */
				bvna.substate = BVNA_SUBSTATE_SEND_REG_REQUEST;
				bvna.wait_timer = BVNA_RADIO_POLLING_MS;

				/* Return to polling mode if retries limit reached */
				if(++bvna.retries >= BVNA_REGISTRATION_SEND_REG_REQ_RETRIES){
					bvna.state = BVNA_STATE_POLLING;
					bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
					bvna.wait_timer = BVNA_RADIO_POLLING_MS;
					bvna_registration_timeout_callback();
					BVNA_LOG("Registration timeout\r\n");
				} else {
					BVNA_LOG("Re-send registration request...!\r\n");
				}
			}
			break;

		case BVNA_SUBSTATE_SEND_STATUS:	/* Send status data at BVNA_BASE_FREQ_HZ */
			if(sx127x_lora_transmit(&ra01, bvna.tx_buffer, bvna.tx_count, 1000) != LORA_OK){
				bvna_error_callback(BVNA_ERROR_TX_TIMEOUT);
				BVNA_LOG("Radio TX timeout\r\n");

				/* Try send again */
				bvna.wait_timer = BVNA_RADIO_POLLING_MS;

				/* Return to polling mode if retries limit reached */
				if(++bvna.retries >= BVNA_TX_ERRORS_FOR_CRITICAL_ERROR){
					bvna.state = BVNA_STATE_POLLING;
					bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
					bvna_error_callback(BVNA_ERROR_TX_ERROR);
					bvna_registration_timeout_callback();
					BVNA_LOG("Radio TX error due to retries limit\r\n");
				}

				break;
			}

			BVNA_LOG("TX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.tx_header->repeater_id,
							bvna.tx_header->device_id, *(bvna.tx_buffer + sizeof(bvna_radio_header_t)));

			/* Status sended, registration successful */
			bvna.state = BVNA_STATE_POLLING;
			bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			bvna.packet_number++;
			bvna_registration_success_callback();
			BVNA_LOG("Device registered!\r\n");
			break;

		default:
			return;
	}
}

static void bvna_polling_process(void){
	switch(bvna.substate){
		case BVNA_SUBSTATE_WAIT_COMMAND:
			bvna.rx_count = sx127x_lora_available(&ra01);
			if(bvna.rx_count){
				bvna.rx_count = sx127x_lora_receive(&ra01, bvna.rx_buffer, bvna.rx_count);
				BVNA_LOG("RX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.rx_header->repeater_id,
						bvna.rx_header->device_id, *(bvna.rx_buffer + sizeof(bvna_radio_header_t)));

				/* Check if it's packet for us */
				if(bvna.rx_header->repeater_id != bvna_nvram.device.id){
					return;
				}

				/* Check if it's repeat command (see device_id, should be 0) */
				if(bvna.rx_header->device_id){
					/* This packet for retranslation */
					return; //TODO: handle packets for retranslation purpose
				}

				/* Check command */
				if(*(bvna.rx_buffer + sizeof(bvna_radio_header_t)) == BVNA_CMD_STATUS){
					bvna_make_status_answer();
					bvna.substate = BVNA_SUBSTATE_SEND_PACKET;
					bvna.wait_timer = 0;
					break;
				}
			}

			/* Reload wait timer */
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			break;

		case BVNA_SUBSTATE_SEND_PACKET:
			if(sx127x_lora_transmit(&ra01, bvna.tx_buffer, bvna.tx_count, 1000) != LORA_OK){
				bvna_error_callback(BVNA_ERROR_TX_TIMEOUT);
				BVNA_LOG("Radio TX timeout\r\n");

				/* Try send again */
				bvna.wait_timer = BVNA_RADIO_POLLING_MS;

				/* Return to polling mode if retries limit reached */
				if(++bvna.retries >= BVNA_TX_ERRORS_FOR_CRITICAL_ERROR){
					bvna_error_callback(BVNA_ERROR_TX_ERROR);
					BVNA_LOG("Radio TX error due to retries limit\r\n");
				} else {
					break;
				}
			}

			BVNA_LOG("TX: REP_ID[0x%.2X] DEV_ID[0x%.2X] CMD[0x%.2X]\r\n", bvna.tx_header->repeater_id,
									bvna.tx_header->device_id, *(bvna.tx_buffer + sizeof(bvna_radio_header_t)));

			/* Packet sended, return to BVNA_SUBSTATE_WAIT_COMMAND */
			bvna.state = BVNA_STATE_POLLING;
			bvna.substate = BVNA_SUBSTATE_WAIT_COMMAND;
			bvna.wait_timer = BVNA_RADIO_POLLING_MS;
			bvna.packet_number++;
			BVNA_LOG("Packet sended!\r\n");
			break;

		default:
			return;
	}
}

static void bvna_make_reg_data_packet(void){
	bvna_radio_header_t *header = (bvna_radio_header_t *)bvna.tx_buffer;
	bvna_radio_reg_request_t *command = (bvna_radio_reg_request_t *)(bvna.tx_buffer + sizeof(bvna_radio_header_t));

	/* Make registration data packet */
	header->device_id = 0;
	header->repeater_id = 0;
	header->packet_number = bvna.packet_number;
	command->command = BVNA_CMD_REGISTRATION;
	command->device_type = BVNA_DEVICE_SLAVE_TYPE;
	bvna.tx_count = sizeof(bvna_radio_header_t) + sizeof(bvna_radio_reg_request_t);
}

static void bvna_make_status_answer(void){
	bvna_radio_header_t *header = (bvna_radio_header_t *)bvna.tx_buffer;
	bvna_radio_status_data_t *status_data = (bvna_radio_status_data_t *)(bvna.tx_buffer + sizeof(bvna_radio_header_t));

	/* Make status data packet */
	header->repeater_id = bvna_nvram.device.id;
	header->device_id = 0;
	header->packet_number = bvna.packet_number++;
	status_data->command = BVNA_CMD_STATUS;
	status_data->battery_soc = 50; //TODO: get real battery soc
	status_data->temperature_c = -10; //TODO: get real temperature
	bvna.tx_count = sizeof(bvna_radio_header_t) + sizeof(bvna_radio_status_data_t);
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

#endif
