#include "shtc3_port.h"
#include "main.h"

/* I2C data structures */
I2C_Handle i2c;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;

static void _i2cInit(void){
    I2C_init();
}

void SHTC3_PL_Init(void){
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
}

void SHTC3_PL_I2C_Init(void){
    if(i2c != NULL){
        return;
    }

    i2c = I2C_open(I2C_0, &i2cParams);
    if (i2c == NULL){
        while (1) {}
    }
}

void SHTC3_PL_I2C_DeInit(void){
    if(i2c != NULL){
        I2C_close(i2c);
        i2c = NULL;
    }
}

void SHTC3_PL_I2C_Write(uint8_t address, uint8_t *data, uint8_t count){
    if(i2c == NULL){
        return;
    }

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf   = data;
    i2cTransaction.writeCount = count;
    i2cTransaction.readBuf    = NULL;
    i2cTransaction.readCount  = 0;
    I2C_transfer(i2c, &i2cTransaction);
}

void SHTC3_PL_I2C_Read(uint8_t address, uint8_t *data, uint8_t count){
    if(i2c == NULL){
        return;
    }

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf   = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf   = data;
    i2cTransaction.readCount = count;
    I2C_transfer(i2c, &i2cTransaction);
}

void SHTC3_PL_Delay(uint32_t milliseconds){
    usleep(milliseconds * 1000);
}
