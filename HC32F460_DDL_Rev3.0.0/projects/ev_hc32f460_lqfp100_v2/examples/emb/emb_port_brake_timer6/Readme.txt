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

================================================================================
功能描述
================================================================================
本样例主要展示EMB模块的端口输入刹车功能。

说明：
EMB检测端口输入电平。
1、满足端口输入电平时，PWM输出指定电平，产生EMB中断，执行EMB中断服务回调函数。
2、EMB中断回调函数内，等待按键释放，然后清除EMB状态，PWM正常输出。

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
1）测试板PA8(TIM6_<t>_PWMA)、PA7(TIM6_<t>_PWMB)引脚与示波器相连，引脚PB1与PA11相连。
2）编译、下载并运行。
3）观察示波器，TIM6_<t>_PWMA、TIM6_<t>_PWMB输出PWM。
4）按下按键K10，不释放。
5）观察示波器，TIM6_<t>_PWMA、TIM6_<t>_PWMB输出低电平。
6）释放按键K10。
7）观察示波器，TIM6_<t>_PWMA、TIM6_<t>_PWMB恢复PWM输出。
8）重复步骤4） ~步骤7）。

================================================================================
注意
================================================================================
无

================================================================================
