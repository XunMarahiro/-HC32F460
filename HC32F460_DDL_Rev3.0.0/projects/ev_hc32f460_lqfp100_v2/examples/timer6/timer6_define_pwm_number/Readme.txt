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
本样例主要展示MCU的TIMER6的输出特定个数的PWM脉冲功能，
说明：
配置Timer6_1，计数时钟为200M/1024，输出PWMA和PWMB
设置Timer6_1的OVF溢出事件为Trigger source 1
设置Timer6_2的OVF溢出事件为Trigger source 0
配置Timer6_2，由Trigger source 1触发向上计数，Timer6_1由Trigger source0 触发停止和清零计数器。
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
1） 打开工程编译并全速运行。
2） 每次程序运行的时候，示波器能看到PA8(Timer6_1 PWMA)和PE8(Timer6_1 PWMB)分别输出6个周期的PWM波形(频率为4Hz)。
================================================================================
注意
================================================================================
无

================================================================================
