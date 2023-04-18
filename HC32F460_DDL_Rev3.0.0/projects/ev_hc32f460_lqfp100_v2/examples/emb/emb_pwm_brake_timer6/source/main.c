/**
 *******************************************************************************
 * @file  emb/emb_pwm_brake_timer6/source/main.c
 * @brief This example demonstrates how to use the PWM same phase brake function
 *        of EMB function.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"

/**
 * @addtogroup HC32F460_DDL_Examples
 * @{
 */

/**
 * @addtogroup EMB_PWM_Brake_TMR6
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* Peripheral register WE/WP selection */
#define LL_PERIPH_SEL                   (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
                                         LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* Key pin definition */
#define KEY_PORT                        (GPIO_PORT_B)
#define KEY_PIN                         (GPIO_PIN_01)

/* TMR6 PWM pin definition */
#define TIM6_PWMA_PORT                  (GPIO_PORT_A)
#define TIM6_PWMA_PIN                   (GPIO_PIN_08)
#define TIM6_PWMA_GPIO_FUNC             (GPIO_FUNC_3)

#define TIM6_PWMB_PORT                  (GPIO_PORT_A)
#define TIM6_PWMB_PIN                   (GPIO_PIN_07)
#define TIM6_PWMB_GPIO_FUNC             (GPIO_FUNC_3)

/* TMR6 unit definition */
#define TMR6_UNIT                       (CM_TMR6_1)
#define TMR6_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR6_1, ENABLE))

/* EMB unit definition */
#define EMB_GROUP                       (CM_EMB0)
#define EMB_FCG_ENABLE()                (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_EMB, ENABLE))

/* EMB interrupt definition */
#define EMB_INT_IRQn                    (INT140_IRQn)
#define EMB_INT_SRC                     (INT_SRC_EMB_GR0)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static __IO en_flag_status_t m_enEmbReleasePwmFlag;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 * @brief  Get key state
 * @param  None
 * @retval An @ref en_flag_status_t enumeration type value.
 */
static en_flag_status_t KEY_State(void)
{
    en_flag_status_t enKeyState = RESET;

    if (PIN_RESET == GPIO_ReadInputPins(KEY_PORT, KEY_PIN)) {
        DDL_DelayMS(50UL);

        if (PIN_RESET == GPIO_ReadInputPins(KEY_PORT, KEY_PIN)) {
            while (PIN_RESET == GPIO_ReadInputPins(KEY_PORT, KEY_PIN)) {
            }
            enKeyState = SET;
        }
    }

    return enKeyState;
}

/**
 * @brief  Get TMR6 clock frequency.
 * @param  None
 * @retval TMR6 clock frequency
 */
static uint32_t TMR6_ClockFreq(void)
{
    return CLK_GetBusClockFreq(CLK_BUS_PCLK0);
}

/**
 * @brief  Configure TMR6 PWM
 * @param  None
 * @retval None
 */
static void TMR6_PwmConfig(void)
{
    stc_timer6_init_t stcTmr6Init;
    stc_tmr6_pwm_init_t stcTmr6PwmInit;

    /* Initialize PWM I/O */
    GPIO_SetFunc(TIM6_PWMA_PORT, TIM6_PWMA_PIN, TIM6_PWMA_GPIO_FUNC);
    GPIO_SetFunc(TIM6_PWMB_PORT, TIM6_PWMB_PIN, TIM6_PWMB_GPIO_FUNC);

    /* Enable TMR6 peripheral clock */
    TMR6_FCG_ENABLE();

    /* TMR6 general count function configuration */
    (void)TMR6_StructInit(&stcTmr6Init);
    stcTmr6Init.sw_count.u32ClockDiv = TMR6_CLK_DIV256;
    stcTmr6Init.u32PeriodValue = TMR6_ClockFreq() / (2UL * (1UL << (uint32_t)(stcTmr6Init.sw_count.u32ClockDiv >> \
                                 TMR6_GCONR_CKDIV_POS))) - 1UL;
    (void)TMR6_Init(TMR6_UNIT, &stcTmr6Init);

    /* Configurate PWM output */
    stcTmr6PwmInit.u32CompareValue = stcTmr6Init.u32PeriodValue / 2UL;
    stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
    stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_LOW;
    stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_HIGH;
    stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_LOW;
    stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_LOW;
    (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_A, &stcTmr6PwmInit);

    stcTmr6PwmInit.u32CompareValue = stcTmr6Init.u32PeriodValue / 2UL;
    stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
    stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_HIGH;
    stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_LOW;
    stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_HIGH;
    stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_HIGH;
    (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_B, &stcTmr6PwmInit);

    /* PWM pin function set */
    TMR6_SetFunc(TMR6_UNIT, TMR6_CH_A, TMR6_PIN_CMP_OUTPUT);
    TMR6_SetFunc(TMR6_UNIT, TMR6_CH_B, TMR6_PIN_CMP_OUTPUT);

    /* PWM output command */
    TMR6_PWM_OutputCmd(TMR6_UNIT, TMR6_CH_A, ENABLE);
    TMR6_PWM_OutputCmd(TMR6_UNIT, TMR6_CH_B, ENABLE);

    /* Start TMR6 count. */
    TMR6_Start(TMR6_UNIT);
}

/**
 * @brief  EMB IRQ hander.
 * @param  None
 * @retval None
 */
void EMB_GR0_IrqHandler(void)
{
    stc_tmr6_pwm_init_t stcTmr6PwmInit;

    if (SET == EMB_GetStatus(EMB_GROUP, EMB_FLAG_PWMS)) {
        /* Wait key pressed */
        while (RESET == KEY_State()) {
        }

        TMR6_Stop(TMR6_UNIT);

        /* Configurate PWM output */
        stcTmr6PwmInit.u32CompareValue = TMR6_GetCompareValue(TMR6_UNIT, TMR6_CMP_REG_A);
        stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
        stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_HIGH;
        stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_LOW;
        (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_A, &stcTmr6PwmInit);

        stcTmr6PwmInit.u32CompareValue = TMR6_GetCompareValue(TMR6_UNIT, TMR6_CMP_REG_B);
        stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
        stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_HIGH;
        stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_HIGH;
        stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_HIGH;
        (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_B, &stcTmr6PwmInit);

        /* Clear count value */
        TMR6_SetCountValue(TMR6_UNIT, 0UL);

        /* Start counter */
        TMR6_Start(TMR6_UNIT);

        /* Clear the EMB PWM status. */
        EMB_ClearStatus(EMB_GROUP, EMB_FLAG_PWMS);

        m_enEmbReleasePwmFlag = SET;
    }
}

/**
 * @brief  Main function of EMB PWM brake
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    stc_emb_tmr6_init_t stcEmbInit;
    stc_tmr6_pwm_init_t stcTmr6PwmInit;
    stc_tmr6_emb_config_t stcTmr6EmbConfig;

    /* MCU Peripheral registers write unprotected */
    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* Configure TMR6 PWM. */
    TMR6_PwmConfig();

    /* Configure TMR6 EMB. */
    (void)TMR6_EMBConfigStructInit(&stcTmr6EmbConfig);
    stcTmr6EmbConfig.u32PinStatus = TMR6_EMB_PIN_LOW;
    (void)TMR6_EMBConfig(TMR6_UNIT, TMR6_CH_A, &stcTmr6EmbConfig);
    (void)TMR6_EMBConfig(TMR6_UNIT, TMR6_CH_B, &stcTmr6EmbConfig);

    /* Enable EMB peripheral clock */
    EMB_FCG_ENABLE();

    /* EMB: initialize */
    (void)EMB_TMR6_StructInit(&stcEmbInit);
    stcEmbInit.stcTmr6.stcTmr6_1.u32PwmState = EMB_TMR6_1_PWM_ENABLE;
    stcEmbInit.stcTmr6.stcTmr6_1.u32PwmLevel = EMB_DETECT_TMR6_1_PWM_BOTH_HIGH;
    (void)EMB_TMR6_Init(EMB_GROUP, &stcEmbInit);

    /* EMB: enable interrupt */
    EMB_IntCmd(EMB_GROUP, EMB_INT_PWMS, ENABLE);

    /* EMB: register IRQ handler && configure NVIC. */
    (void)INTC_ShareIrqCmd(EMB_INT_SRC, ENABLE);
    NVIC_ClearPendingIRQ(EMB_INT_IRQn);
    NVIC_SetPriority(EMB_INT_IRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(EMB_INT_IRQn);

    /* MCU Peripheral registers write protected */
    LL_PERIPH_WP(LL_PERIPH_SEL);

    for (;;) {
        /* Wait key pressed */
        while (RESET == KEY_State()) {
        }

        /* Stop TMR6 */
        TMR6_Stop(TMR6_UNIT);

        /* Configurate PWM output */
        stcTmr6PwmInit.u32CompareValue = TMR6_GetCompareValue(TMR6_UNIT, TMR6_CMP_REG_A);
        stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
        stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_HIGH;
        stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_LOW;
        (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_A, &stcTmr6PwmInit);

        stcTmr6PwmInit.u32CompareValue = TMR6_GetCompareValue(TMR6_UNIT, TMR6_CMP_REG_B);
        stcTmr6PwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
        stcTmr6PwmInit.u32PeriodMatchPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32CompareMatchPolarity = TMR6_PWM_HIGH;
        stcTmr6PwmInit.u32StopPolarity = TMR6_PWM_LOW;
        stcTmr6PwmInit.u32StartPolarity = TMR6_PWM_LOW;
        (void)TMR6_PWM_Init(TMR6_UNIT, TMR6_CH_B, &stcTmr6PwmInit);

        /* Clear count value */
        TMR6_SetCountValue(TMR6_UNIT, 0UL);

        m_enEmbReleasePwmFlag = RESET;

        /* Start counter */
        TMR6_Start(TMR6_UNIT);

        /* Wait that EMB release PWM */
        while (RESET == m_enEmbReleasePwmFlag) {
        }
    }
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
