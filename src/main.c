#include <stdio.h>
#include "M480.h"
#include "NuMicro.h"
#include "lsm6dso32.h"
#include "spi_reg.h"
#include "viccom.h"
#include "wdt.h"
#include "math.h"
#include "node.pb.h"
#include "comms.h"

viccom_t comms;
NodeState state;


void SYS_Init(void)
{

        /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    //CLK->PWRCTL   = (CLK->PWRCTL   & ~(0x0004000FUL)) | 0x00000014UL;
    //CLK->PLLCTL   = (CLK->PLLCTL   & ~(0x000FFFFFUL)) | 0x0008421EUL;
    //CLK->CLKDIV0  = (CLK->CLKDIV0  & ~(0xFFFFFFFFUL)) | 0x00000000UL;
    //CLK->CLKDIV1  = (CLK->CLKDIV1  & ~(0x000000FFUL)) | 0x00000000UL;
    //CLK->CLKDIV2  = (CLK->CLKDIV2  & ~(0xFF00000FUL)) | 0x00000000UL;
    //CLK->CLKDIV3  = (CLK->CLKDIV3  & ~(0x0000FFFFUL)) | 0x00000000UL;
    //CLK->CLKDIV4  = (CLK->CLKDIV4  & ~(0x00FFFFFFUL)) | 0x00000000UL;
    //CLK->PCLKDIV  = (CLK->PCLKDIV  & ~(0x00000077UL)) | 0x00000000UL;
    //CLK->CLKSEL0  = (CLK->CLKSEL0  & ~(0x0033013FUL)) | 0x00F3013AUL;
    //CLK->CLKSEL1  = (CLK->CLKSEL1  & ~(0xFF777703UL)) | 0xBF777703UL;
    //CLK->CLKSEL2  = (CLK->CLKSEL2  & ~(0x00003FFFUL)) | 0x000003ABUL;
    //CLK->CLKSEL3  = (CLK->CLKSEL3  & ~(0xFFF33103UL)) | 0xFF00003FUL;
    //CLK->AHBCLK   = (CLK->AHBCLK   & ~(0x000193CEUL)) | 0x00008006UL;
    //CLK->APBCLK0  = (CLK->APBCLK0  & ~(0x3FFFF7FFUL)) | 0x00012000UL;
    //CLK->APBCLK1  = (CLK->APBCLK1  & ~(0x9ECF1011UL)) | 0x00000000UL;
    //CLK->CLKOCTL  = (CLK->CLKOCTL  & ~(0x0000007FUL)) | 0x00000000UL;
    //SysTick->CTRL = (SysTick->CTRL & ~(0x00000005UL)) | 0x00000000UL;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFUL)) | 0x0008421EUL;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV1 | CLK_PCLKDIV_PCLK1DIV1);

    /* Enable IP clock */
    CLK_EnableModuleClock(FMCIDLE_MODULE);
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));


    SPI0->CTL |= SPI_CTL_CLKPOL_Msk;



    //SYS->GPA_MFPH = 0x00000000UL;
    //SYS->GPA_MFPL = 0x00000444UL;
    //SYS->GPB_MFPH = 0x00000000UL;
    //SYS->GPB_MFPL = 0x000F0000UL;
    //SYS->GPC_MFPL = 0x00000000UL;
    //SYS->GPF_MFPL = 0x000033EEUL;

    /* If the macros do not exist in your project, please refer to the corresponding header file in Header folder of the tool package */
    SYS->GPA_MFPH = SYS_GPA_MFPH_PA13MFP_GPIO;
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI;
    SYS->GPB_MFPH = 0x00000000;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB4MFP_INT1;
    SYS->GPC_MFPL = 0x00000000;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF3MFP_UART0_TXD | SYS_GPF_MFPL_PF2MFP_UART0_RXD | SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;

    //initialize uart and vicom
    uint32_t txEnable = 0;
    UART_T *uart = (UART_T*)UART0;
    uint8_t address = 1;
    VICCOM_nuvoton_init(&comms, uart,&txEnable,  address);

    //initialize spi and accelerometer
    lsm6dso32_init();

}

int main (void)
{
    SYS_Init();
    while(1) {
        COMMS_task();
    }
}