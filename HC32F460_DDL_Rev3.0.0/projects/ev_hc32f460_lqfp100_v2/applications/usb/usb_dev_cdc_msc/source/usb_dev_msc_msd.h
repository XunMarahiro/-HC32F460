/**
 *******************************************************************************
 * @file  usb/usb_dev_cdc_msc/source/usb_dev_msc_msd.h
 * @brief Header file for usb_dev_msc_msd.c
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
#ifndef __USB_DEV_MSC_MSD_H__
#define __USB_DEV_MSC_MSD_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_core.h"

/**
 * @addtogroup HC32F460_DDL_Applications
 * @{
 */

/**
 * @addtogroup USB_Dev_Msc
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern const int8_t msc_inquirydata[];

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEV_MSC_MSD_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
