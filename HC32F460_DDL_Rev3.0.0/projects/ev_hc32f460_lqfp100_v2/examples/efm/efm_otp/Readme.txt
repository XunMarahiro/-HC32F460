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
本样例为EFM模块中一次性可编程字节（otp）样例。对otp块14先进行擦除，编程，
然后锁定该区域，通过对该区域进行编程判断锁定是否成功。
若锁定成功，则第二次编程失败，蓝灯灭，红灯亮。

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
2）启动IDE的下载和调试功能，全速运行；
3）蓝灯亮，按下按键K1，LED蓝灯灭，LED红灯亮（表示锁定成功, 编程失败）。



================================================================================
注意
================================================================================
当使用keil工程，需在option->Debug->Settings->Flash Download里添加HC32F460 otp flash

================================================================================
