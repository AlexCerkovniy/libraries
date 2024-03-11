#include "sx1276_lora.h"
#include "sx1276_lora_misc.h"
#include "sx1276_port.h"

extern tLoRaSettings LoRaSettings;

void sx1276_lora_set_rf_frequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1276LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1276LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    sx1276_write_burst( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );
}

uint32_t sx1276_lora_get_rf_frequency( void )
{
    sx1276_read_burst( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );
    LoRaSettings.RFFrequency = ( ( uint32_t )SX1276LR->RegFrfMsb << 16 ) | ( ( uint32_t )SX1276LR->RegFrfMid << 8 ) | ( ( uint32_t )SX1276LR->RegFrfLsb );
    LoRaSettings.RFFrequency = ( uint32_t )( ( double )LoRaSettings.RFFrequency * ( double )FREQ_STEP );

    return LoRaSettings.RFFrequency;
}

void sx1276_lora_set_rf_power( int8_t power )
{
    sx1276_read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    sx1276_read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    sx1276_write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
    LoRaSettings.Power = power;
}

int8_t sx1276_lora_get_rf_power( void )
{
    sx1276_read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    sx1276_read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 )
        {
            LoRaSettings.Power = 5 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            LoRaSettings.Power = 2 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        LoRaSettings.Power = -1 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return LoRaSettings.Power;
}

void sx1276_lora_set_signal_bandwidth( uint8_t bw )
{
    sx1276_read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    sx1276_write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

uint8_t sx1276_lora_get_signal_bandwidth( void )
{
    sx1276_read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    return LoRaSettings.SignalBw;
}

void sx1276_lora_set_spreading_factor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
    	sx1276_lora_set_nb_trig_peaks( 5 );
    }
    else
    {
    	sx1276_lora_set_nb_trig_peaks( 3 );
    }

    sx1276_read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    sx1276_write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );
    LoRaSettings.SpreadingFactor = factor;
}

uint8_t sx1276_lora_get_spreading_factor( void )
{
    sx1276_read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    LoRaSettings.SpreadingFactor = ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return LoRaSettings.SpreadingFactor;
}

void sx1276_lora_set_error_coding( uint8_t value )
{
    sx1276_read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    sx1276_write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

uint8_t sx1276_lora_get_error_coding( void )
{
    sx1276_read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    return LoRaSettings.ErrorCoding;
}

void sx1276_lora_set_packet_crc_on( bool enable )
{
    sx1276_read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    sx1276_write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

bool sx1276_lora_get_packet_crc_on( void )
{
    sx1276_read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    return LoRaSettings.CrcOn;
}

void sx1276_lora_set_tx_packet_timeout( uint32_t value )
{
    LoRaSettings.TxPacketTimeout = value;
}

uint32_t sx1276_lora_get_tx_packet_timeout( void )
{
    return LoRaSettings.TxPacketTimeout;
}

void sx1276_lora_set_rx_packet_timeout( uint32_t value )
{
    LoRaSettings.RxPacketTimeout = value;
}

uint32_t sx1276_lora_get_rx_packet_timeout( void )
{
    return LoRaSettings.RxPacketTimeout;
}

void sx1276_lora_set_payload_length( uint8_t value )
{
    SX1276LR->RegPayloadLength = value;
    sx1276_write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

uint8_t sx1276_lora_get_payload_length( void )
{
    sx1276_read( REG_LR_PAYLOADLENGTH, &SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = SX1276LR->RegPayloadLength;
    return LoRaSettings.PayloadLength;
}

void sx1276_lora_set_pa_20dbm( bool enable )
{
    sx1276_read( REG_LR_PADAC, &SX1276LR->RegPaDac );
    sx1276_read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( enable == true )
        {
            SX1276LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1276LR->RegPaDac = 0x84;
    }
    sx1276_write( REG_LR_PADAC, SX1276LR->RegPaDac );
}

bool sx1276_lora_get_pa_20dbm( void )
{
    sx1276_read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    return ( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

void sx1276_lora_set_pa_output( uint8_t outputPin )
{
    sx1276_read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    sx1276_write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
}

uint8_t sx1276_lora_get_pa_output( void )
{
    sx1276_read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    return SX1276LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}

void sx1276_lora_set_pa_ramp( uint8_t value )
{
    sx1276_read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    SX1276LR->RegPaRamp = ( SX1276LR->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    sx1276_write( REG_LR_PARAMP, SX1276LR->RegPaRamp );
}

uint8_t sx1276_lora_get_pa_ramp( void )
{
    sx1276_read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    return SX1276LR->RegPaRamp & ~RFLR_PARAMP_MASK;
}

void sx1276_lora_set_symbol_timeout( uint16_t value )
{
    sx1276_read_burst( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );

    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1276LR->RegSymbTimeoutLsb = value & 0xFF;
    sx1276_write_burst( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
}

uint16_t sx1276_lora_get_symbol_timeout( void )
{
    sx1276_read_burst( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
    return ( ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | SX1276LR->RegSymbTimeoutLsb;
}

void sx1276_lora_set_low_datarate_optimize( bool enable )
{
    sx1276_read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    SX1276LR->RegModemConfig3 = ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    sx1276_write( REG_LR_MODEMCONFIG3, SX1276LR->RegModemConfig3 );
}

bool sx1276_lora_get_low_datarate_optimize( void )
{
    sx1276_read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    return ( ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

void sx1276_lora_set_preamble_length( uint16_t value )
{
    sx1276_read_burst( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );

    SX1276LR->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    SX1276LR->RegPreambleLsb = value & 0xFF;
    sx1276_write_burst( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
}

uint16_t sx1276_lora_get_preamble_length( void )
{
    sx1276_read_burst( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
    return ( ( SX1276LR->RegPreambleMsb & 0x00FF ) << 8 ) | SX1276LR->RegPreambleLsb;
}

void sx1276_lora_set_nb_trig_peaks( uint8_t value )
{
    sx1276_read( 0x31, &SX1276LR->RegDetectOptimize );
    SX1276LR->RegDetectOptimize = ( SX1276LR->RegDetectOptimize & 0xF8 ) | value;
    sx1276_write( 0x31, SX1276LR->RegDetectOptimize );
}

uint8_t sx1276_lora_get_nb_trig_peaks( void )
{
    sx1276_read( 0x31, &SX1276LR->RegDetectOptimize );
    return ( SX1276LR->RegDetectOptimize & 0x07 );
}
