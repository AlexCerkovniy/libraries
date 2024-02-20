#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

/* General definitions for LP8 sensor */
#include "senseair_lp8_config.h"

#define LP8_READY_TIMEOUT                   (210U)
#define LP8_UART_TIMEOUT                    (500U)

typedef enum {
    /* Used only when a previous sensor state is unavailable; e.g. after a fresh boot-up or after previous
     * re-calibrations where the noise-suppression filters have been reset.
     */
    LP8_INITIAL_CMD = 0x10,

    /* The default operating mode, reusing previous sensor states for noise suppression and passing
     * background calibration references
     */
    LP8_SEQUENTAL_CMD = 0x20,

    /* Requires a stable ambient environment free from any CO2, i.e. concentration is 0 ppm. Makes a new
     * measurement and sets a new baseline offset from the noisy and non-filtered internal signal reference
     */
    LP8_ZERO_CAL_NON_FILTERED_CMD = 0x40,

    /* Requires a stable ambient environment free from any CO2, i.e. concentration is 0 ppm. Makes a new
     * measurement and sets a new baseline offset from the noise-filtered internal signal reference from the
     * new and old measurement.
     */
    LP8_ZERO_CAL_FILTERED_CMD = 0x41,

    /* Requires a stable ambient environment free from any CO2, i.e. concentration is 0 ppm. Makes a new
     * measurement and sets a new baseline offset from the noisy and non-filtered internal signal reference.
     * It then overwrites the internal historic noise parameter references set after this measurement to be
     * the same as the current measured raw value.
     */
    LP8_ZERO_CAL_NON_FILTERED_RESET_NOISE_FILTERS_CMD = 0x42,

    /* Requires a stable ambient environment free from any CO2, i.e. concentration is 0 ppm. Makes a new
     * measurement and sets a new baseline offset from the noise-filtered internal signal reference. It then
     * overwrites the internal historic noise parameter references set after this measurement to be the
     * same as the current measured raw value.
     */
    LP8_ZERO_CAL_FILTERED_RESET_NOISE_FILTERS_CMD = 0x43,

    /* Requires a stable ambient environment in fresh air, i.e. concentration is near-400 ppm. LP8 makes a
     * new measurement and sets a new baseline offset by the difference between the noisy and nonfiltered
     * (but pressure-compensated) internal signal reference and the predefined fresh air reference.
     */
    LP8_BACKGROUND_CAL_NON_FILTERED_CMD = 0x50,

    /* Requires a stable ambient environment in fresh air, i.e. concentration is near-400 ppm. LP8 makes a
     * new measurement and sets a new baseline offset from the difference between the noise-filtered (but
     * pressure-compensated) internal signal reference and the predefined fresh air reference.
     */
    LP8_BACKGROUND_CAL_FILTERED_CMD = 0x51,

    /* Requires a stable ambient environment in fresh air, i.e. concentration is near-400 ppm. LP8 makes a
     * new measurement and sets a new baseline offset from the difference between the noisy and nonfiltered (but pressure-compensated) internal signal reference and the predefined fresh air reference.
     * It then overwrites the internal historic noise parameter references set after this measurement to be
     * the same as the current measured raw value.
     */
    LP8_BACKGROUND_CAL_NON_FILTERED_RESET_NOISE_FILTERS_CMD = 0x52,

    /* Requires a stable ambient environment in fresh air, i.e. concentration is near-400 ppm. LP8 makes a
     * new measurement and sets a new baseline offset by the difference between the noise-filtered
     * (pressure-compensated) internal signal reference and the predefined fresh air reference. It then
     * overwrites the internal historic noise parameter references set after this measurement to be the
     * same as the current measured raw value
     */
    LP8_BACKGROUND_CAL_FILTERED_RESET_NOISE_FILTERS_CMD = 0x53,

    /* Informs the LP8 to perform an ABC calibration based on the stored baseline reference continuously
     * passed back and forth within the sensor states. A new offset is set based from the stored reference
     * value in Sensor State and its assumed correlation to 400ppm.
     */
    LP8_PERORM_ABC_FILTERED_CMD = 0x70,

    /* Informs the LP8 to perform an ABC calibration based on the stored baseline reference continuously
     * passed back and forth within the sensor states. A new offset is set based from the stored reference
     * value in Sensor State and its assumed correlation to 400ppm. It then overwrites the internal historic
     * noise parameter reference set after this measurement to be the same as the current measured raw
     * value.
     */
    LP8_PERORM_ABC_FILTERED_RESET_NOISE_FILTERS_CMD = 0x71,

    /* Empty defined command, nothing to do */
    LP8_EMPTY_CMD = 0
} lp8_control_cmd_t;

typedef enum {
    LP8_WAIT_NONE = 0U,
    LP8_WAIT_READY_LOW,
    LP8_WAIT_READY_HIGH,
    LP8_WAIT_UART_ANSWER
} lp8_wait_event_t;

typedef enum {
    LP8_SUSPEND = 0U,
    LP8_POWER_ON,
    LP8_PERFORM_RAM_WRITE,
    LP8_PERFORN_RAM_READ,
    LP8_HANDLE_RECEIVED_DATA
} lp8_state_t;

typedef enum {
    LP8_ERROR_NONE = 0,
    LP8_ERROR_RDY_TIMEOUT,
    LP8_ERROR_UART_ANSWER_TIMEOUT
} lp8_error_t;

typedef struct __attribute__((packed)) {
    uint8_t address;
    uint8_t function;
    uint8_t register_high;
    uint8_t register_low;
    uint8_t count;
} lp8_modbus_header_t;

typedef struct __attribute__((packed)) {
    uint8_t calculation_control;
    uint8_t sensor_state[23];
    int16_t host_pressure;
} lp8_write_registers_t;

typedef union __attribute__((packed)) {
    uint8_t byte[4];
    struct{
        /* Byte 3 */
        uint8_t filtered_channel        :4;
        uint8_t reserved3               :4;

        /* Byte 2 */
        uint8_t unfiltered_channel      :4;
        uint8_t reserved2               :4;

        /* Byte 1 */
        uint8_t vcap1_low               :1;
        uint8_t vcap2_low               :1;
        uint8_t adc_error               :1;
        uint8_t reserved1               :1;
        uint8_t param_override          :4;

        /* Byte 0 */
        uint8_t fatal_error             :1;
        uint8_t reserved0               :1;
        uint8_t alg_error               :1;
        uint8_t calibration             :1;
        uint8_t self_diagnostic         :1;
        uint8_t out_of_range            :1;
        uint8_t memory                  :1;
        uint8_t warm_up                 :1;
    } parsed;
} lp8_error_registers_t;

typedef struct __attribute__((packed)) {
    lp8_write_registers_t write;
    uint8_t co2_uncompensated_unfiltered_hi;
    uint8_t co2_uncompensated_unfiltered_lo;
    uint8_t co2_compensated_unfiltered_hi;
    uint8_t co2_compensated_unfiltered_lo;
    uint8_t temperature_hi;
    uint8_t temperature_lo;
    uint8_t vcap1_hi;
    uint8_t vcap1_lo;
    uint8_t vcap2_hi;
    uint8_t vcap2_lo;
    lp8_error_registers_t error;
    uint8_t co2_uncompensated_filtered_hi;
    uint8_t co2_uncompensated_filtered_lo;
    uint8_t co2_compensated_filtered_hi;
    uint8_t co2_compensated_filtered_lo;
} lp8_registers_t;

typedef struct {
    uint16_t uncompensated_unfiltered;
    uint16_t compensated_unfiltered;
    uint16_t uncompensated_filtered;
    uint16_t compensated_filtered;
} lp8_co2_data_t;

void LP8_Init(void);
void LP8_Main(void);
void LP8_Tick(uint32_t period);
void LP8_Enable(void);
void LP8_Disable(void);
lp8_co2_data_t *LP8_GetCO2(void);
uint16_t LP8_GetVDD(void);
void LP8_Command(lp8_control_cmd_t command);
void LP8_RegisterUpdateCallback(void (*callback)(void));
void LP8_RegisterErrorCallback(void (*callback)(lp8_error_t error));
