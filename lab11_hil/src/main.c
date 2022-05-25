#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "heartbeat_task.h"

#include "uart.h"
#include "cmd_task.h"

#include "mod_inputs.h"
#include "mod_fault_detection.h"
#include "mod_state_machine.h"
#include "mod_outputs.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialise scheduler
    osKernelInitialize();

    // Initialise hardware modules
    uart_init();

    // Initialise task modules
    heartbeat_task_init();
    cmd_task_init();
    module_inputs_init();
    module_fault_detect_init();
    module_state_machine_init();
    module_outputs_init();

    // Start scheduler
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for(;/*ever*/;);
}

/*
    * System Clock Configuration:
    *           System Clock source     = PLL (HSI)
    *           SYSCLK(Hz)              = 100000000
    *           HCLK(Hz)                = 100000000
    *           AHB Prescaler           = 1
    *           APB1 Prescaler          = 2
    *           APB2 Prescaler          = 1
    *           HSI Frequency(Hz)       = 16000000
    *           PLL_M                   = 16
    *           PLL_N                   = 400
    *           PLL_P                   = 4
    *           PLL_Q                   = 7
    *           VDD(V)                  = 3.3
    *           Main regulator output voltage   = Scale2 mode
    *           Flash Latency(WS)       = 3
*/
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 400;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Error_Handler(void)
{
    while(1)
    {
        /* Bury head in sand */
    }
}
