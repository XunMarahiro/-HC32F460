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
本样例为如何配置DMAC为通道重置模式，先通过软件触发DMA传输3个数据，然后通过按键k10
触发通道重置。
重置后传输次数按源地址方式，地址按源地址方式。
本样例中源地址设置的为重复功能，剩余次数为：TC - RPTZ_SIZE = （20-7）= 13。


================================================================================
测试环境
================================================================================
测试用板:
---------------------
EV_F460_LQ100_Rev2.0

辅助工具:
---------------------
无

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）打开工程并重新编译；
2）全速运行，按下k10（触发通道重置）；
3）可观察到蓝灯亮。


================================================================================
注意
================================================================================
无

================================================================================
