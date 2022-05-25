#include "stm32f4xx_hal.h"

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
   	/* Timer 2: Initialise the onboard LED for heartbeat on PA5 */
    if(htim->Instance == TIM2)
    {
        /* Enable peripheral clocks */
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* Configure PA5 in AF pushpull mode with TIM2 alternate function */
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.Pin = GPIO_PIN_5;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    /* TODO: Enable ADC1 clock */
    /* TODO: Enable GPIOA clock */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* TODO: Initialise PA0 and PA1 in analog mode, no pullup */
    static GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_Initstructure.Mode = GPIO_MODE_ANALOG;
    GPIO_Initstructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_Initstructure);
    //GPIO_Init_PA0.Pin = GPIO_PIN_0;
    //GPIO_Init_PA0.Mode = GPIO_MODE_ANALOG;
    //GPIO_Init_PA0.Pull = GPIO_NOPULL;
    //HAL_GPIO_Init(GPIOA, &GPIO_PIN_0);
}