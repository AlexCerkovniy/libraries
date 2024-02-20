#include "senseair_lp8.h"
#include "senseair_lp8_port.h"
#include "string.h"

static struct {
    bool enabled;                           /* Module state */
    bool initial_start;                     /* Initial start flag: no sensor info available for restore */
    uint32_t timer;                         /* Timer (count down, for time intervals measurement) */
    lp8_state_t state;                      /* Sensor state */
    lp8_wait_event_t wait;                  /* Wait state */
    lp8_error_t error;                      /* Last error state */
    uint8_t control_cmd;                    /* Custom [lp8_control_cmd_t] calculation control state, applied at next measurement cycle */
    uint8_t tx_data[64];                    /* UART transmit data container */
    uint8_t tx_count;                       /* Transmit data size */
    uint8_t rx_data[64];                    /* UART receive data container */
    uint8_t rx_count;                       /* Expected answer bytes size */
    lp8_registers_t registers;              /* Sensor registers values */
    lp8_co2_data_t co2;                     /* Collected CO2 values from last measurement */

    void (*updateCallback)(void);
    void (*errorCallback)(lp8_error_t error);
} lp8;

static void _request(uint8_t *tx_data, uint8_t tx_count, uint8_t *rx_data, uint8_t rx_count);
static uint16_t _crc16(uint8_t *data, uint16_t count);
static void _errorHandler(lp8_error_t error);

void LP8_Init(void){
    LP8_PL_Init();

    lp8.enabled = false;
    lp8.initial_start = true;
    lp8.control_cmd = LP8_EMPTY_CMD;
    lp8.error = LP8_ERROR_NONE;
}

void LP8_Main(void){
    static lp8_modbus_header_t *request = (lp8_modbus_header_t *)lp8.tx_data;
    static uint16_t crc16;

    if(!lp8.enabled){
        return;
    }

    /* Waiting some events */
    switch(lp8.wait){
        case LP8_WAIT_NONE:
            break;

        case LP8_WAIT_READY_LOW:
            if(lp8.timer == 0){
               _errorHandler(LP8_ERROR_RDY_TIMEOUT);
               return;
            }

            if(LP8_PL_RDY_Read() == false){
                LP8_PL_Delay(2);
                lp8.wait = LP8_WAIT_NONE;
                lp8.timer = 0;
                break;
            }
            return;

        case LP8_WAIT_READY_HIGH:
            if(lp8.timer == 0){
                _errorHandler(LP8_ERROR_RDY_TIMEOUT);
                return;
            }

            /* Check RDY pin */
            if(LP8_PL_RDY_Read() == true){
                LP8_PL_Delay(2);
                lp8.wait = LP8_WAIT_NONE;
                lp8.timer = 0;
                break;
            }
            return;

        case LP8_WAIT_UART_ANSWER:
            if(lp8.timer == 0){
                _errorHandler(LP8_ERROR_UART_ANSWER_TIMEOUT);
                return;
            }

            /* Poll UART */
            if(LP8_PL_UART_Rx(lp8.rx_data, lp8.rx_count)){
                /* Extract received data CRC */
                crc16 = lp8.rx_data[lp8.rx_count - 1];
                crc16 = (crc16 << 8) | lp8.rx_data[lp8.rx_count - 2];

                /* Compare it to calculated CRC */
                if(_crc16(lp8.rx_data, lp8.rx_count - 2) == crc16){
                    lp8.initial_start = false;
                    lp8.wait = LP8_WAIT_NONE;
                    lp8.timer = 0;

                    /* If next state is read, then need to wait sensor processing */
                    if(lp8.state == LP8_PERFORN_RAM_READ){
                        lp8.wait = LP8_WAIT_READY_HIGH;
                        lp8.timer = LP8_READY_TIMEOUT;
                    }
                    break;
                }
            }
            return;

        default:
            break;
    }

    /* Global sensor state machine */
    if(lp8.timer == 0){
        switch(lp8.state){
            case LP8_POWER_ON:
                lp8.state = LP8_PERFORM_RAM_WRITE;
                lp8.wait = LP8_WAIT_READY_LOW;
                lp8.timer = LP8_READY_TIMEOUT;

                /* Enable VBB sensor power */
                LP8_PL_VBB_EN_Set(true);
                LP8_PL_UART_Init(LP8_UART_BAUDRATE);
                LP8_PL_Delay(LP8_POWER_UP_TIME);
                break;

            case LP8_PERFORM_RAM_WRITE:
                if(lp8.initial_start){
                    lp8.registers.write.calculation_control = LP8_INITIAL_CMD;
                }
                else{
                    if(lp8.control_cmd != LP8_EMPTY_CMD){
                        lp8.registers.write.calculation_control = lp8.control_cmd;
                    }
                    else{
                        lp8.registers.write.calculation_control = LP8_SEQUENTAL_CMD;
                    }
                }

                /* Update pressure compensation value */
            #if LP8_PRESSURE_COMPENSATION
                lp8.registers.write.host_pressure = 10124;
            #endif

                /* Prepare Modbus RTU request packet */
                request->address = LP8_UART_ADDRESS;
                request->function = 0x41;
                request->register_high = 0x00;
                request->register_low = 0x80;
            #if LP8_PRESSURE_COMPENSATION
                request->count = sizeof(lp8_write_registers_t);
            #else
                request->count = sizeof(lp8_write_registers_t) - 2;
            #endif
                memcpy(lp8.tx_data + sizeof(lp8_modbus_header_t), &lp8.registers.write, request->count);

                /* Append CRC16 */
                lp8.tx_count = sizeof(lp8_modbus_header_t) + request->count;
                crc16 = _crc16(lp8.tx_data, lp8.tx_count);
                lp8.tx_data[lp8.tx_count] = crc16 & 0xFF;
                lp8.tx_data[lp8.tx_count + 1] = (crc16 >> 8) & 0xFF;
                lp8.tx_count += sizeof(crc16);

                /* Send request */
                _request(lp8.tx_data, lp8.tx_count, lp8.rx_data, 4);

                /* Go to next state */
                lp8.state = LP8_PERFORN_RAM_READ;
                break;

            case LP8_PERFORN_RAM_READ:
                request->address = LP8_UART_ADDRESS;
                request->function = 0x44;
                request->register_high = 0x00;
                request->register_low = 0x80;
                request->count = 44;

                /* Append CRC16 */
                lp8.tx_count = sizeof(lp8_modbus_header_t);
                crc16 = _crc16(lp8.tx_data, lp8.tx_count);
                lp8.tx_data[lp8.tx_count] = crc16 & 0xFF;
                lp8.tx_data[lp8.tx_count + 1] = (crc16 >> 8) & 0xFF;
                lp8.tx_count += sizeof(crc16);

                /* Send request */
                _request(lp8.tx_data, lp8.tx_count, lp8.rx_data, request->count + 5);

                /* Go to next state */
                lp8.state = LP8_HANDLE_RECEIVED_DATA;
                break;

            case LP8_HANDLE_RECEIVED_DATA:
                memcpy(&lp8.registers, lp8.rx_data + 3, 44);

                /* Fill CO2 structure */
                lp8.co2.uncompensated_unfiltered = ((uint16_t)lp8.registers.co2_uncompensated_unfiltered_hi << 8) | lp8.registers.co2_uncompensated_unfiltered_lo;
                lp8.co2.uncompensated_filtered = ((uint16_t)lp8.registers.co2_uncompensated_filtered_hi << 8) | lp8.registers.co2_uncompensated_filtered_lo;
                lp8.co2.compensated_unfiltered = ((uint16_t)lp8.registers.co2_compensated_unfiltered_hi << 8) | lp8.registers.co2_compensated_unfiltered_lo;
                lp8.co2.compensated_filtered = ((uint16_t)lp8.registers.co2_compensated_filtered_hi << 8) | lp8.registers.co2_compensated_filtered_lo;

                /* Reset custom command */
                lp8.control_cmd = LP8_EMPTY_CMD;

                /* Update callback */
                if(lp8.updateCallback){
                    lp8.updateCallback();
                }

                /* Disable VBB sensor power and go to suspend mode */
                lp8.state = LP8_SUSPEND;
                lp8.timer = LP8_READ_PERIOD;
                LP8_PL_VBB_EN_Set(false);
                LP8_PL_UART_DeInit();
                break;

            case LP8_SUSPEND:
                if(lp8.enabled){
                    lp8.state = LP8_POWER_ON;
                }
                break;

        }
    }
}

void LP8_Tick(uint32_t period){
    if(!lp8.enabled){
        return;
    }

    if(lp8.timer){
        if(lp8.timer < period){
            lp8.timer = 0;
        }
        else{
            lp8.timer -= period;
        }
    }
}

void LP8_Enable(void){
    if(lp8.state == LP8_SUSPEND){
        lp8.state = LP8_POWER_ON;
        lp8.timer = 0;
    }

    lp8.enabled = true;
}

void LP8_Disable(void){
    lp8.enabled = false;
}

lp8_co2_data_t *LP8_GetCO2(void){
    return &lp8.co2;
}

uint16_t LP8_GetVDD(void){
    return ((uint16_t)lp8.registers.vcap2_hi << 8) | lp8.registers.vcap2_lo;
}

void LP8_Command(lp8_control_cmd_t command){
    lp8.control_cmd = command;
    if(lp8.timer > 0){
        if(lp8.state == LP8_POWER_ON || lp8.state == LP8_SUSPEND){
            lp8.timer = 0;
            lp8.state = LP8_POWER_ON;
        }
    }
}

void LP8_RegisterUpdateCallback(void (*callback)(void)){
    lp8.updateCallback = callback;
}

void LP8_RegisterErrorCallback(void (*callback)(lp8_error_t error)){
    lp8.errorCallback = callback;
}

static void _request(uint8_t *tx_data, uint8_t tx_count, uint8_t *rx_data, uint8_t rx_count){
    /* Send request data to sensor */
    LP8_PL_UART_Tx(tx_data, tx_count);

    /* Set-up wait answer event */
    lp8.wait = LP8_WAIT_UART_ANSWER;
    lp8.timer = LP8_UART_TIMEOUT;
    lp8.rx_count = rx_count;
}

static void _errorHandler(lp8_error_t error){
    lp8.state = LP8_SUSPEND;
    lp8.wait = LP8_WAIT_NONE;
    lp8.timer = LP8_READ_SHIFT_IN_CASE_OF_ERROR;
    lp8.error = error;
    LP8_PL_VBB_EN_Set(false);
    LP8_PL_UART_DeInit();

    if(lp8.errorCallback){
        lp8.errorCallback(error);
    }
}

static uint16_t _crc16(uint8_t *data, uint16_t count){
    static const uint16_t table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    uint16_t crc = 0xFFFF;
    uint8_t xor = 0;

    while(count--){
        xor = (*data++) ^ crc;
        crc >>= 8;
        crc ^= table[xor];
    }

    return crc;
}
