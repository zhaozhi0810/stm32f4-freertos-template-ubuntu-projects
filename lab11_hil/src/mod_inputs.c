#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "mod_inputs.h"
#include "mod_fault_detection.h"

/* Misc. module specific varables */
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfigADC;
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;

/* Structures defined for the fault detection thread */
static osThreadId_t _inputs_thread_id;
static osThreadAttr_t _inputs_attr = 
{
    .name = "inputsModule",
    .priority = osPriorityHigh //Set initial thread priority
};

///////////////////////////////////// THREAD FUNCTIONS /////////////////////////////////////
/* Task to manage fault detection module */
void module_inputs_task(void *argument)
{
    UNUSED(argument);
    while(1)
    {
        // Update state of fault detection module
        module_inputs_update();
        // Wait 500ms between updates
        osDelay(500);
    }
}

/* Update sensor measuments */
// ADC0: Pressure
// ADC1: Temperature
// PC9: Estop
void module_inputs_update(void)
{
    // create local error flag to track faults
    errorCode localErrorFlag = FAULT_HEALTHY;
    
    //Multichannel ADC Conv.
    /* TODO: Start the ADC */
    HAL_ADC_Start(&hadc1);
    /* TODO: Poll and measure pressure (rank 1) */
    sConfigADC.Rank = 1;
    HAL_ADC_PollForConversion(&hadc1, 0xff);
    /* TODO: Get the return value from the ADC conversion */
    uint32_t V_ADC_in = HAL_ADC_GetValue(&hadc1);
    /* TODO: Compute the 4-20mA current */
    float Vp = 3.3 / 4095 * V_ADC_in;
    float Ip = Vp / 125;
        /* TODO: Check for over / under current faults. Set 'localErrorFlag' using the fault codes in 'fault_codes.h' if fault detected */
        if (Ip > 20) {
            localErrorFlag = FAULT_ESTOP;
        }

        else if (Ip < 4) {
            localErrorFlag = FAULT_ESTOP;
        }
    
        /* TODO: Compute the pressure */
   float P = 0 + (Ip - 4) / (20 - 4) * 2500;
        /* TODO: Push pressure to fault detection module */
            //fault detection module
   module_fault_detect_set_pressure(P);

    /* NOTE: The inputs module does not check for pressure > 400kPa. This is done in the fault detection module */


    /* TODO: Poll and measure temperature (rank 2) */
     sConfigADC.Rank = 2;
     HAL_ADC_PollForConversion(&hadc1, 0xff);
    /* TODO: Get the return value from the ADC conversion */
      V_ADC_in = HAL_ADC_GetValue(&hadc1);
    /* TODO: Compute the 4-20mA current */
      Vp = 3.3 / 4095 * V_ADC_in;
      Ip = Vp / 125;
     /* TODO: Check for over / under current faults. Set 'localErrorFlag' using the fault codes in 'fault_codes.h' if fault detected */
         if (Ip > 20) {
             localErrorFlag = FAULT_ESTOP;
         }

         else if (Ip < 4) {
             localErrorFlag = FAULT_ESTOP;
         }
     /* TODO: Compute the temperature */
     float T = 0 + (Ip - 4) / (20 - 4) * 100;
    /* TODO: Push temperature to fault detection module */
    module_fault_detect_set_temperature(T);
         /* NOTE: The inputs module does not check for temperature > 60C. This is done in the fault detection module */


         /* TODO: Push any detected hardware errors to fault detection module */
    module_fault_detect_set_error(localErrorFlag);


         /* TODO: Stop the ADC */
         HAL_ADC_Stop(&hadc1);

    /* Read Estop state and push states to fault detection module */
    /* NOTE: The inputs module does not generate an Estop fault. This is done in the fault detection module */
         
         if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 0) {
             module_fault_detect_set_estop(FAULT);
         }
         else {
             module_fault_detect_set_estop(HEALTHY);
         }
}


///////////////////////////////////// INITIALISATION FUNCTIONS /////////////////////////////////////

/* initialise inputs module as a thread */
void module_inputs_init(void)
{
    // initialise hardware
    module_inputs_configure_hardware();
    // create thread for kernel to manage
    _inputs_thread_id = osThreadNew(module_inputs_task, NULL, &_inputs_attr);
    _is_init = 1;
    _is_running = 1;
}

/* configure hardware for inputs module */
// ADC0: Pressure
// ADC1: Temperature
// PC9: Estop
void module_inputs_configure_hardware(void)
{
    if (!_is_init)
    {
        /* TODO: Configure ADC1 instance for:
                   1 div2 prescaler,
                   2 12-bit resolution, 
                   3 right data aligned, 
                   4 scan conversion mode enabled,
                   5 continuous conversion mode disabled,
                   6 End of Conversion flag selected for single conversion,
                   7 Two conversions performed.
            Note:   Config parameters of the ADC_InitTypeDef not mentioned above
                    are not required in this lab            */
         hadc1.Instance = ADC1;
         hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
         hadc1.Init.Resolution = ADC_RESOLUTION_12B;
         hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
         hadc1.Init.ScanConvMode = ENABLE;
         hadc1.Init.ContinuousConvMode = DISABLE;
         hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
         hadc1.Init.NbrOfConversion = 2;
        /* TODO: Configure the ADC to:
                    use channel 0,
                    sequence rank of 1,
                    480 cycle sample time,
                    offset of 0.                            */
         sConfigADC.Channel = ADC_CHANNEL_0;
         sConfigADC.Rank = 1;
         sConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
         sConfigADC.Offset = 0;
        /* TODO: Initialise ADC */  
         HAL_ADC_Init(&hadc1);
        /* TODO: Configure the ADC channel 0. Note: this should be sequence rank 1 */
         HAL_ADC_ConfigChannel(&hadc1,&sConfigADC);
        /* TODO: Configure the ADC channel 1. Note: this should be sequence rank 2 */  
         sConfigADC.Channel = ADC_CHANNEL_1;
         sConfigADC.Rank = 2;
         HAL_ADC_ConfigChannel(&hadc1, &sConfigADC);
        
        /* TODO: Configure estop pin. You're on your own for this one */
         __HAL_RCC_GPIOC_CLK_ENABLE();
         GPIO_InitTypeDef GPIO_Init_PC9;
         GPIO_Init_PC9.Pin = GPIO_PIN_9;
         /* TODO: Initialise estop pin */
        GPIO_Init_PC9.Mode = GPIO_MODE_INPUT;
        GPIO_Init_PC9.Pull = GPIO_PULLUP;
         //GPIO_Init_PC9.Speed = GPIO_SPEED_FREQ_LOW;
         //GPIO_Init_PC9.Alternate = GPIO_AF1_TIM2;
         /* TODO: Call the HAL function to initialise GPIOA pin 9 */
         HAL_GPIO_Init(GPIOA, &GPIO_Init_PC9);
    }
}


///////////////////////////////////// ADDITIONAL FUNCTIONS /////////////////////////////////////
/* start inputs module */
void module_inputs_start(void)
{
    if(!_is_running && _is_init)
    {
        osThreadResume(_inputs_thread_id);
        _is_running = 1;
    }
    printf("Inputs module on\n");
}

/* stop inputs module */
void module_inputs_stop(void)
{
    if(_is_running && _is_init)
    {
        osThreadSuspend(_inputs_thread_id);
        _is_running = 0;
    }
    printf("Inputs module off\n");
}
