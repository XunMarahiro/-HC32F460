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
本例程实现了CRC16和CRC32的计算和校验。

说明：
1）硬件CRC写入指定的数据，得到CRC结果；
2）硬件CRC写入1）中指定的数据和CRC结果
3）根据CRC结果状态，判断CRC计算结果是否正确

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
1）打开工程，编译、下载并运行程序；
2）观察LED蓝灯常亮，表示硬件CRC计算结果正确；

================================================================================
注意
================================================================================
LED状态说明：
   a）蓝色LED灯亮，表示硬件CRC计算结果正确。
   b）红色LED灯亮，表示硬件CRC计算结果错误。

================================================================================
