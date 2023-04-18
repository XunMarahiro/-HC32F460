/**
 *******************************************************************************
 * @file  functional_safety/iec60730_class_b/source/test_impl_item/test_impl_wdt.h
 * @brief This file contains all the functions prototypes of the watchdog test.
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

#ifndef __TEST_IMPL_WDT_H__
#define __TEST_IMPL_WDT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stl_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup STL_IEC60730_Application
 * @{
 */

/**
 * @addtogroup Test_Implement_WDT
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
*******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup Test_Implement_WDT_Global_Functions
 * @{
 */
uint32_t STL_WdtStartupTest(void);
uint32_t STL_WdtRuntimeInit(void);
uint32_t STL_WdtRuntimeFeed(void);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
{
#endif


#endif /* __TEST_IMPL_WDT_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
