#include "sx1276_lora.h"
#include "sx1276_lora_misc.h"
#include "sx1276_port.h"

/* https://github.com/SMotlaq/LoRa/blob/master/LoRa/LoRa.c */

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
sx127x_lora_t LoRaSettings =
{
    433000000,        // RFFrequency
    13,               // Power
    9,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    7,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             // CrcOn [0: OFF, 1: ON]
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    128,              // PayloadLength (used for implicit header mode)
};

/*!
 * SX1276 registers variable
 */
uint8_t SX1276Regs[0x70];

/*!
 * SX1276 LoRa registers variable
 */
tSX1276LR* SX1276LR;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;

/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

void sx1276_lora_init( void )
{
    RFLRState = RFLR_STATE_IDLE;
    SX1276LR = ( tSX1276LR* )SX1276Regs;

    sx1276_io_init();
    sx1276_lora_reset();
    sx1276_lora_set_defaults();

    /* Enable lora mode */
    sx1276_lora_set_op_mode( RFLR_OPMODE_SLEEP );
    sx1276_delay_ms(5);

	SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
	sx1276_write( REG_LR_OPMODE, SX1276LR->RegOpMode );

	sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );
									// RxDone               RxTimeout                   FhssChangeChannel           CadDone
	SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
									// CadDetected          ModeReady
	SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
	sx1276_write_burst( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

	sx1276_read_burst( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );

    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;

    sx1276_write_burst( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );

    // set the RF settings
    sx1276_lora_set_rf_frequency( LoRaSettings.RFFrequency );
    sx1276_lora_set_spreading_factor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    sx1276_lora_set_error_coding( LoRaSettings.ErrorCoding );
    sx1276_lora_set_packet_crc_on( LoRaSettings.CrcOn );
    sx1276_lora_set_signal_bandwidth( LoRaSettings.SignalBw );

    sx1276_lora_set_symbol_timeout( 0x3FF );
    sx1276_lora_set_payload_length( LoRaSettings.PayloadLength );
    sx1276_lora_set_low_datarate_optimize( true );

    sx1276_lora_set_pa_output( RFLR_PACONFIG_PASELECT_RFO );
    sx1276_lora_set_pa_20dbm( false );
	LoRaSettings.Power = 14;
	sx1276_lora_set_rf_power( LoRaSettings.Power );

//#if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) )
//    if( LoRaSettings.RFFrequency > 860000000 )
//    {
//        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
//        SX1276LoRaSetPa20dBm( false );
//        LoRaSettings.Power = 14;
//        SX1276LoRaSetRFPower( LoRaSettings.Power );
//    }
//    else
//    {
//        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
//        SX1276LoRaSetPa20dBm( true );
//        LoRaSettings.Power = 20;
//        SX1276LoRaSetRFPower( LoRaSettings.Power );
//    }
//#elif( MODULE_SX1276RF1JAS == 1 )
//    if( LoRaSettings.RFFrequency > 860000000 )
//    {
//        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
//        SX1276LoRaSetPa20dBm( true );
//        LoRaSettings.Power = 20;
//        SX1276LoRaSetRFPower( LoRaSettings.Power );
//    }
//    else
//    {
//        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
//        SX1276LoRaSetPa20dBm( false );
//        LoRaSettings.Power = 14;
//        SX1276LoRaSetRFPower( LoRaSettings.Power );
//    }
//#endif

    sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );
}

void sx1276_lora_set_defaults( void )
{
    // REMARK: See SX1276 datasheet for modified default values.

    sx1276_read( REG_LR_VERSION, &SX1276LR->RegVersion );

    sx1276_read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
    sx1276_read_burst( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
    sx1276_read( REG_LR_VERSION, &SX1276LR->RegVersion );
    sx1276_read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
}

void sx1276_lora_reset( void )
{
	sx1276_set_reset(0);
	sx1276_delay_ms(2);
    sx1276_set_reset(1);
    sx1276_delay_ms(7);
}

void sx1276_lora_set_op_mode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            //RXTX( antennaSwitchTxOn ); // Antenna switch control //TODO: investigate
        }
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

        sx1276_write( REG_LR_OPMODE, SX1276LR->RegOpMode );
    }
}

uint8_t sx1276_lora_get_op_mode( void )
{
    sx1276_read( REG_LR_OPMODE, &SX1276LR->RegOpMode );

    return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t sx1276_lora_read_rx_gain( void )
{
    sx1276_read( REG_LR_LNA, &SX1276LR->RegLna );
    return( SX1276LR->RegLna >> 5 ) & 0x07;
}

double sx1276_lora_read_rssi( void )
{
    // Reads the RSSI value
    sx1276_read( REG_LR_RSSIVALUE, &SX1276LR->RegRssiValue );

    if( LoRaSettings.RFFrequency < 860000000 )  // LF
    {
        return RSSI_OFFSET_LF + ( double )SX1276LR->RegRssiValue;
    }
    else
    {
        return RSSI_OFFSET_HF + ( double )SX1276LR->RegRssiValue;
    }
}

uint8_t sx1276_lora_get_packet_rx_gain( void )
{
    return RxGain;
}

int8_t sx1276_lora_get_packet_snr( void )
{
    return RxPacketSnrEstimate;
}

double sx1276_lora_get_packet_rssi( void )
{
    return RxPacketRssiValue;
}

void sx1276_lora_start_rx( void )
{
	sx1276_lora_set_rf_state( RFLR_STATE_RX_INIT );
}

void sx1276_lora_get_rx_packet( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void sx1276_lora_set_tx_packet( const void *buffer, uint16_t size )
{
    TxPacketSize = size;
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize );

    RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t sx1276_lora_get_rf_state( void )
{
    return RFLRState;
}

void sx1276_lora_set_rf_state( uint8_t state )
{
    RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY,
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t sx1276_lora_process( void )
{
    uint32_t result = RF_BUSY;

    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:

        sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );

        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        sx1276_write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );
        sx1276_write( REG_LR_HOPPERIOD, 255 );

                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        sx1276_write_burst( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );


		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
		sx1276_write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );

		sx1276_lora_set_op_mode( RFLR_OPMODE_RECEIVER );

        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = sx1276_get_tick( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
    	sx1276_read(REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags);

        if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE /*DIO0 == 1*/ ) // RxDone
        {
            RxTimeoutTimer = sx1276_get_tick( );

            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL /*DIO2 == 1*/ ) // FHSS Changed Channel
        {
            RxTimeoutTimer = sx1276_get_tick( );

            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = sx1276_lora_read_rx_gain( );
        }
        break;
    case RFLR_STATE_RX_DONE:
        sx1276_read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
        if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            RFLRState = RFLR_STATE_RX_RUNNING;
            break;
        }


		uint8_t rxSnrEstimate;
		sx1276_read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
		if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
		{
			// Invert and divide by 4
			RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
			RxPacketSnrEstimate = -RxPacketSnrEstimate;
		}
		else
		{
			// Divide by 4
			RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
		}

        sx1276_read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );

        if( LoRaSettings.RFFrequency < 860000000 )  // LF
        {
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + ( ( double )SX1276LR->RegPktRssiValue ) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + ( 1.0666 * ( ( double )SX1276LR->RegPktRssiValue ) );
            }
        }
        else                                        // HF
        {
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_HF + ( ( double )SX1276LR->RegPktRssiValue ) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = RSSI_OFFSET_HF + ( 1.0666 * ( ( double )SX1276LR->RegPktRssiValue ) );
            }
        }


		sx1276_read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );

		sx1276_read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
		RxPacketSize = SX1276LR->RegNbRxBytes;
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
		sx1276_write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
		sx1276_read_burst( REG_LR_FIFO, RFBuffer, SX1276LR->RegNbRxBytes );
        RFLRState = RFLR_STATE_RX_RUNNING;
        result = RF_RX_DONE;
        break;

    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;

    case RFLR_STATE_TX_INIT:
        sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );

		SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
									RFLR_IRQFLAGS_RXDONE |
									RFLR_IRQFLAGS_PAYLOADCRCERROR |
									RFLR_IRQFLAGS_VALIDHEADER |
									//RFLR_IRQFLAGS_TXDONE |
									RFLR_IRQFLAGS_CADDONE |
									RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
									RFLR_IRQFLAGS_CADDETECTED;
        sx1276_write( REG_LR_HOPPERIOD, 0 );
        sx1276_write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1276LR->RegPayloadLength = TxPacketSize;
        sx1276_write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );

        SX1276LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        sx1276_write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );

        SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
        sx1276_write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );

        // Write payload buffer to LORA modem
        sx1276_write_burst(REG_LR_FIFO, RFBuffer, SX1276LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
        sx1276_write_burst( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

        sx1276_lora_set_op_mode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
    	sx1276_read(REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags);

    	if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE /*DIO0 == 1*/ ) // TxDone
        {
            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;
        }
        if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL /*DIO2 == 1*/ ) // FHSS Changed Channel
        {
            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;

    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;

    case RFLR_STATE_CAD_INIT:
        sx1276_lora_set_op_mode( RFLR_OPMODE_STANDBY );

        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        sx1276_write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        sx1276_write_burst( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

        sx1276_lora_set_op_mode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;

    case RFLR_STATE_CAD_RUNNING:
    	sx1276_read(REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags);

        if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_CADDONE /*DIO3 == 1*/ ) //CAD Done interrupt
        {
            // Clear Irq
            sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if(SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_CADDETECTED /*DIO4 == 1*/ ) // CAD Detected interrupt
            {
                // Clear Irq
                sx1276_write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            }
            else
            {
                // The device goes in Standby Mode automatically
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }
        break;

    default:
        break;
    }
    return result;
}
