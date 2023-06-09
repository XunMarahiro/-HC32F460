﻿================================================================================
                                样例使用说明
================================================================================
版本历史
Date        Version     Author      IAR     MDK     GCC     Description
2022-03-31  1.0         CDT         7.70    5.16    8.3.1   First version
================================================================================
平台说明
================================================================================
GCC工程，由Eclipse IDE外挂GNU-ARM Toolchain，再结合pyOCD GDB Server实现工程的编译、
链接和调试。在用Eclipse导入工程后，请将xxxx_PyOCDDebug中pyocd-gdbserver和SVD文件
设置为正确的路径；请将xxxx_PyOCDDownload中pyocd设置为正确的路径。注意，这些路径不
能包含非英文字符。


功能描述
================================================================================
本样例主要展示MCU的TIMER6的PWM输出功能(使能有效周期，每隔6个周期产生一次中断和触发事件）
说明：
配置timer6，计数时钟为200M/1024

Timer6通过AOS功能触发ADC1启动转换，触发事件为SCMA的计数比较匹配事件EVT_SRC_TMR6_1_SCMP_A。
================================================================================
测试环境
================================================================================
测试用板:
---------------------
EV_F460_LQ100_Rev2.0

辅助工具:
---------------------
示波器

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）使用示波器测试PA8(Timer6_1 PWMA)和 PB8的输出波形。
2）打开工程编译并全速运行。
3）观察示波器波形，PA8翻转表示计数匹配，PB8的翻转表示ADC扫描完成一次。
   当宏定义SCMA_ValidPeriod==0U时，计数匹配1次，ADC扫描一次；
   当宏定义SCMA_ValidPeriod==1U时，计数匹配6次，ADC扫描一次。

================================================================================
注意
================================================================================
无

================================================================================
