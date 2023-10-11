#include "M480.h"
#include "NuMicro.h"
#include <stdint.h>
#include "gpio.h"
#include "lsm6dso32.h"
#include "lsm6dso32_reg.h"
#include <stdlib.h>
#include "string.h"
#include <stdbool.h>
#include "../vicproto2/c/proto/common.pb.h"
#include "viccom.h"
#include "pb_encode.h"

#define SS  PA13
#define SPI_BUFFER_SIZE 100
uint8_t txData[SPI_BUFFER_SIZE];
uint8_t rxData[SPI_BUFFER_SIZE];
volatile uint32_t txDataIdx = 0;
volatile uint32_t rxDataIdx = 0;
uint32_t spiTransferCount = 0;

uint8_t readReg(uint8_t reg);
void writeReg(uint8_t reg, uint8_t val);

#define SPI_TX_DMA_CH 2
#define SPI_RX_DMA_CH 3




void lsm6dso32_init()
{
    //SPI0, master mode, SPI mode 0, data witdth of 16, bus clock of 6MHz
    volatile int rate = SPI_Open(SPI0, SPI_MASTER, SPI_MODE_3, 8, 6000000);

    //MISO needs to pull down and be set as an input
    GPIO_SetMode(PA, BIT1, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT1, GPIO_PUSEL_PULL_DOWN);

    //enable chip select pin
    GPIO_SetMode(PA, BIT13, GPIO_MODE_OUTPUT);

    //enable imu interrupt pin
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PB, BIT4, GPIO_PUSEL_PULL_UP);
    PB->INTEN |= GPIO_INTEN_FLIEN4_Msk;

    //chip select idles high
    SS = 1;

    PDMA_Open(PDMA,(1 << SPI_TX_DMA_CH) | (1 << SPI_RX_DMA_CH));
   
    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Increasing
        Destination = SPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_TX_DMA_CH, (uint32_t)txData, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    //enable for now to know transfer occurred. 
    PDMA->DSCT[SPI_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = SPI0->RX
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)rxData, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;


    //enable accelerometer
    volatile uint8_t whoami = readReg(WHO_AM_I);
    
    // Enable accelerometer 833hz +-4g
    writeReg(CTRL1_XL, HIGH_PERFORMANCE_833_ACCEL);

    // Enable gyroscope 833hz +-250dps
    writeReg(CTRL2_G, HIGH_PERFORMANCE_833_GYRO);

    // FIFO bypass mode (FIFO disabled)
    writeReg(FIFO_CTRL4, FIFO_BYPASS_MODE);

    // enable interrupt output
    writeReg(INT1_CTRL, INT1_DRDY_XL|INT1_DRDY_G);

    //interrupt pins are active low and if multiple byte read, register address increments
    writeReg(CTRL3_C, H_LACTIVE_LOW|IF_INC_EN);

    //enable NVIC interrupt
    NVIC_EnableIRQ(EINT1_IRQn);

}

void lsm6dso32_getData(uint8_t* buffer)
{
    readRange(OUTX_L_G, buffer, 12);
}

void spiTransfer()
{
    txDataIdx = 0;
    rxDataIdx = 0;
    SS = 0;

    int32_t error = 0;

    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA,SPI_TX_DMA_CH, PDMA_WIDTH_8, spiTransferCount);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA,SPI_RX_DMA_CH, PDMA_WIDTH_8, spiTransferCount);

    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetTransferMode(PDMA,SPI_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);

    SPI_TRIGGER_TX_RX_PDMA(SPI0);

        while(1)
        {
            /* Get interrupt status */
            uint32_t status = PDMA_GET_INT_STATUS(PDMA);
            /* Check the PDMA transfer done interrupt flag */
            if(status & PDMA_INTSTS_TDIF_Msk)
            {

                /* Check the PDMA transfer done flags */
                if((PDMA_GET_TD_STS(PDMA) & ((1 << SPI_TX_DMA_CH) | (1 << SPI_RX_DMA_CH))) ==
                        ((1 << SPI_TX_DMA_CH) | (1 << SPI_RX_DMA_CH)))
                {

                    /* Clear the PDMA transfer done flags */
                    PDMA_CLR_TD_FLAG(PDMA,(1 << SPI_TX_DMA_CH) | (1 << SPI_RX_DMA_CH));

                    /* Disable SPI master's PDMA transfer function */
                    SPI_DISABLE_TX_RX_PDMA(SPI0);
                    break;
                }
            }
            /* Check the DMA transfer abort interrupt flag */
            if(status & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                uint32_t abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,abort);
                error = 1;
                break;
            }
            /* Check the DMA time-out interrupt flag */
            if(status & (PDMA_INTSTS_REQTOF0_Msk|PDMA_INTSTS_REQTOF1_Msk))
            {
                /* Clear the time-out flag */
                PDMA->INTSTS = status & (PDMA_INTSTS_REQTOF0_Msk|PDMA_INTSTS_REQTOF1_Msk);
                error = 1;
                break;
            }

        }

    //set chip select back to 1
    SS = 1;
}

uint8_t readReg(uint8_t reg)
{
    memset(txData, 0, sizeof(txData));
    memset(rxData, 0, sizeof(rxData));

    txData[0] = reg | 0x80; // set read bit

    spiTransferCount = 2;
    spiTransfer();
    return rxData[1];
}

void readRange(uint8_t reg, uint8_t* readData, int count)
{
    memset(txData, 0, sizeof(txData));
    memset(rxData, 0, sizeof(rxData));

    txData[0] = reg | 0x80; // set read bit

    spiTransferCount = count+1;
    spiTransfer();
    memcpy(readData, rxData+1, count);
}


void writeReg(uint8_t reg, uint8_t val) {
    memset(txData, 0, sizeof(txData));
    memset(rxData, 0, sizeof(rxData));

    txData[0] = reg & ~(0x80); // set write bit

    txData[0] = reg;
    txData[1] = val;

    spiTransferCount = 2;
    spiTransfer();
}

