#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rtc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_tim.h"

/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */

void mydelay(uint8_t i);
int dht11_get_inform();
void button();
void decoding();
static void set_indicator(uint8_t number);
static void systick_config(void);

int value = 0;
uint8_t out_flag = 0;
uint8_t bit[40] = {0};
uint8_t temp;
uint8_t hum;
uint8_t humdr;
uint8_t tempdr;
uint8_t hash;

void my_delay(uint32_t d)
{
    uint32_t t = LL_TIM_GetCounter(TIM2);
    while ((LL_TIM_GetCounter(TIM2) - t) < d)
        ; //delay for "d" us
}

static void systick_config(void)
{
    LL_InitTick(48000000, 800);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}

/*
 * Handler for system timer
 * Count up to counter_top then switch led
 * (to make blinking more visible)
 */
void SysTick_Handler(void)
{
    static uint8_t cs = 0;
    switch (cs)
    {
    case 0:
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
        set_indicator(value % 10);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
        cs = 1;
        break;
    case 1:
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
        set_indicator((value / 10) % 10);
        if (((value / 10) % 10 != 0) || ((value / 100) % 10 != 0) || ((value / 1000) % 10 != 0))
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
        cs = 2;
        break;
    case 2:
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
        set_indicator((value / 100) % 10);
        if (((value / 100) % 10 != 0) || ((value / 1000) % 10 != 0))
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
        cs = 3;
        break;
    case 3:
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
        set_indicator((value / 1000) % 10);
        if ((value / 1000) % 10 != 0)
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
        cs = 0;
        break;
    }
}

static void exti_config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
    /*
     * Setting interrupts
     */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
}
void EXTI0_1_IRQHandler(void)
{
    my_delay(10000);
    if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) == 1)
        out_flag = (out_flag + 1) % 2;
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

int dht11_get_inform()
{
    uint8_t i = 0;
    uint32_t t = 0, time = 0, t1;
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
    my_delay(18000); // Starting data transmission
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
    my_delay(60);
    if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3) != 0)
        return 1;
    my_delay(80);
    if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3) != 1)
        return 2;

    while (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3) == 1)
        ;

    for (i = 0; i < 40; i++) //data transmission
    {
        if (LL_TIM_GetCounter(TIM2) > 0xAFFFFFFF)
            LL_TIM_GenerateEvent_UPDATE(TIM2);
        while (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3) == 0)
            ;
        t = LL_TIM_GetCounter(TIM2);
        while (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3) == 1)
            ;
        time = LL_TIM_GetCounter(TIM2) - t;
        if ((time <= 13) || (time >= 80))
            return 4;
        if ((time > 13) && (time < 35))
            bit[i] = 0;
        if ((time > 60) && (time < 80))
            bit[i] = 1;
    }
    return 0;
}

void decoding()
{
    uint8_t buf = 0;
    for (int j = 0; j < 20; j++)
    {
        buf = bit[j];
        bit[j] = bit[39 - j];
        bit[39 - j] = buf;
    }

    int i = 0;
    for (i = 0; i < 8; i++)
        hum = hum | (*(bit + i) << i);
    for (i = 8; i < 16; i++)
        humdr = humdr | (*(bit + i) << (i - 8));
    for (i = 16; i < 24; i++)
        temp = temp | (*(bit + i) << (i - 16));
    for (i = 24; i < 32; i++)
        tempdr = tempdr | (*(bit + i) << (i - 24));
    for (i = 32; i < 40; i++)
        hash = hash | (*(bit + i) << (i - 32));
}

static void set_indicator(uint8_t number)
{
    /*
     * Put all pins for indicator together (for segments only)
     */
    static uint32_t mask = LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 |
                           LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_4 |
                           LL_GPIO_PIN_5 | LL_GPIO_PIN_15;
    static const uint32_t decoder[] = {
        LL_GPIO_PIN_5 | LL_GPIO_PIN_8 | LL_GPIO_PIN_6 | LL_GPIO_PIN_11 |
            LL_GPIO_PIN_15 | LL_GPIO_PIN_4, // 0
        LL_GPIO_PIN_11 | LL_GPIO_PIN_15,     // 1
        LL_GPIO_PIN_6 | LL_GPIO_PIN_8 | LL_GPIO_PIN_12 | LL_GPIO_PIN_4 |
            LL_GPIO_PIN_15, // 2
        LL_GPIO_PIN_6 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_4 |
            LL_GPIO_PIN_15,                                                                                // 3
        LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15 | LL_GPIO_PIN_5,                                 // 4
        LL_GPIO_PIN_6 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_5 | LL_GPIO_PIN_4,                 //5
        LL_GPIO_PIN_8 | LL_GPIO_PIN_4 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //6
        LL_GPIO_PIN_11 | LL_GPIO_PIN_15 | LL_GPIO_PIN_4,                                                  // 7
        LL_GPIO_PIN_5 | LL_GPIO_PIN_8 | LL_GPIO_PIN_6 | LL_GPIO_PIN_11 |
            LL_GPIO_PIN_15 | LL_GPIO_PIN_4 | LL_GPIO_PIN_12, //8
        LL_GPIO_PIN_5 | LL_GPIO_PIN_12 | LL_GPIO_PIN_6 | LL_GPIO_PIN_11 |
            LL_GPIO_PIN_15 | LL_GPIO_PIN_4};
    const uint8_t max_num = sizeof(decoder) / sizeof(uint32_t);
    uint32_t port_state = 0;

    /*
     * Read current state and do not change pins that are not related to
     * indicator (that is done by using masking)
     */
    port_state = LL_GPIO_ReadOutputPort(GPIOA);
    /*
     * Example:
     * 01100101 <= Input
     * mask = 111 (pins allowed to be changed)
     * ~mask = 11111000 (inverted mask sets remaing bits to one)
     * result = result & ~mask (zero only first three bits)
     * result = result | 001 (modify first three bits)
     * result -> 01100001
     */
    port_state = (port_state & ~mask) | decoder[number % max_num];
    LL_GPIO_WriteOutputPort(GPIOA, port_state);
    return;
}


static void rcc_config()
{
    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    /* Enable HSI and wait for activation*/
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1)
        ;

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1)
        ;

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
        ;

    /* Set APB1 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    /* Update CMSIS variable (which can be updated also
     * through SystemCoreClockUpdate function) */
    /* Set FLASH latency */

    SystemCoreClock = 48000000;
}

/*
 * Clock on GPIOC and set pin with Blue led connected
 */
static void gpio_config(void)
{
    /*
     * Configure LEDs
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);

    /*
     * Init port for indicator
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    /*
     * Init port for USER button
     */
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    return;
}

static void timers_config(void)
{
    /*
      * Setup timer
      */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 47);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM2, 0xFFFFFFFF);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    return;
}

static void rtc_config(void)
{
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1)
        ;

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_EnableBkUpAccess();

    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI); //low speed
    LL_RCC_EnableRTC();
    /*
     * Set up time and date
     */
    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnableInitMode(RTC);
    while (!LL_RTC_IsActiveFlag_INIT(RTC))
        ;

    LL_RTC_SetAsynchPrescaler(RTC, 0x7F);
    LL_RTC_SetSynchPrescaler(RTC, 0x0300);

    LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
    LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_FRIDAY, 1,
                       LL_RTC_MONTH_MARCH, 2019);
    LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, 11, 22, 00);

    LL_RTC_DisableInitMode(RTC);
    LL_RTC_EnableWriteProtection(RTC);

    /*
     * Set up alarm
     */
    LL_RTC_DisableWriteProtection(RTC);

    LL_RTC_ALMA_Disable(RTC);
    while (!LL_RTC_IsActiveFlag_ALRAW(RTC))
        ;
    LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_ALL);
    //LL_RTC_ALMA_ConfigTime(RTC, LL_RTC_ALMA_TIME_FORMAT_AM, 00, 00, 00);
    LL_RTC_ALMA_Enable(RTC);
    LL_RTC_EnableIT_ALRA(RTC);

    LL_RTC_EnableWriteProtection(RTC);

    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_17);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_17);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_17);
    return;
}

void RTC_IRQHandler(void)
{
    hum = 0;
    temp = 0;
    humdr = 0;
    tempdr = 0;
    hash = 0;
    uint8_t err = dht11_get_inform();
    if (err != 0)
        value = 1000 * err + 666; // error code on indicator;
    else
    {
        decoding();
        if (out_flag == 0)
            value = temp;
        if (out_flag == 1)
            value = hum;
    }
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
    LL_RTC_ClearFlag_ALRA(RTC);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_17);
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
    LL_RTC_ClearFlag_ALRA(RTC);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_17);
}

int main(void)
{
    rcc_config();
    timers_config();
    gpio_config();
    rtc_config();
    systick_config();
    exti_config();

    while (1);
    return 0;
}
