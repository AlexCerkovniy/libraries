// Registers' Address
#define DEVID 					(0x00)	//Device ID
#define THRESH_TAP				(0x1D)	//Tap threshold
#define OFFX					(0x1E)	//X-axis offset
#define OFFY					(0x1F)	//Y-axis offset
#define OFFZ					(0x20)	//Z-axis offset
#define DUR						(0x21)	//Tap duration
#define LATENT					(0x22)	//Tap latency
#define WINDOW					(0x23)	//Tap window
#define THRESH_ACT				(0x24)	//Activity threshold
#define THRESH_INACT			(0x25)	//Inactivity threshold
#define TIME_INAT				(0x26)	//Inactivity time
#define ACT_INACT_CTL			(0x27)	//Axis enable control for activity and inactivity detection
#define THRESH_FF 				(0x28)	//Free-fall threshold
#define TIME_FF					(0x29)	//Free-fall time
#define TAP_AXES				(0x2A)	//Axis control for single tap/double tap
#define ACT_TAP_STATUS			(0x2B)	//Source of single tap/double tap
#define BW_RATE					(0x2C)	//Data rate and power mode control
#define POWER_CTL 				(0x2D)	//Power-saving features control
#define INT_ENABLE				(0x2E)	//Interrupt enable control
#define INT_MAP					(0x2F)	//Interrupt mapping control
#define INT_SOURCE				(0x30)	//Source of interrupts
#define DATA_FORMAT 			(0x31)	//Data format control
#define DATAX0					(0x32)
#define DATAX1					(0x33)
#define DATAY0					(0x34)
#define DATAY1					(0x35)
#define DATAZ0					(0x36)
#define DATAZ1					(0x37)
#define FIFO_CTL 				(0x38)	//FIFO control
#define FIFO_STATUS 			(0x39)	//FIFO status

// Init. Definitions
#define SPIMODE_3WIRE 			1
#define SPIMODE_4WIRE 			0

#define LPMODE_NORMAL 0
#define LPMODE_LOWPOWER 1

#define BWRATE_6_25 	6
#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12
#define BWRATE_800		13
#define BWRATE_1600   	14
#define BWRATE_3200   	15

#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12

#define INT_ACTIVEHIGH 		(0x00)
#define INT_ACTIVELOW  		(0x01)

#define RESOLUTION_FULL  1
#define RESOLUTION_10BIT 0

#define JUSTIFY_MSB 	1
#define JUSTIFY_SIGNED  0

#define	SLEEP_RATE_1HZ 3
#define SLEEP_RATE_2HZ 2
#define SLEEP_RATE_4HZ 1
#define SLEEP_RATE_8HZ 0

#define RANGE_2G  0
#define RANGE_4G  1
#define RANGE_8G  2
#define RANGE_16G 3

#define AUTOSLEEPON  1
#define AUTOSLEEPOFF 0

#define LINKMODEON  1
#define LINKMODEOFF 0
