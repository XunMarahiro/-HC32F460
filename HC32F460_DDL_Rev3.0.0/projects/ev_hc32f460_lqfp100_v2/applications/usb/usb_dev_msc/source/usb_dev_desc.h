/**
 *******************************************************************************
 * @file  usb/usb_dev_msc/source/usb_dev_desc.h
 * @brief Head file for usb_dev_desc.c
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
#ifndef __USB_DEV_DESC_H__
#define __USB_DEV_DESC_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_def.h"
#include "stdint.h"

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
#define USB_DEVICE_DESCRIPTOR_TYPE              (0x01U)
#define USB_CFG_DESCRIPTOR_TYPE                 (0x02U)
#define USB_STRING_DESCRIPTOR_TYPE              (0x03U)
#define USB_INTERFACE_DESCRIPTOR_TYPE           (0x04U)
#define USB_ENDPOINT_DESCRIPTOR_TYPE            (0x05U)
#define USB_SIZ_DEVICE_DESC                     (18U)
#define USB_SIZ_STRING_LANGID                   (4U)

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern usb_dev_desc_func user_desc;

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

#endif /* __USB_DEV_DESC_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
