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
本样例展示ICG模块中HRC功能。

说明：
本样例设置HRC启动频率为16M，通过PE0端口输出时钟频率，可连接示波器查看；并每秒闪
烁LED_RED提示工作状态。

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
1）打开并重新编译工程，启动IDE的下载程序功能，关闭IDE下载界面；
2）观察LED_RED闪烁；
3) 连接示波器到PE0端口，查看输出的HRC时钟频率为16Mhz。

================================================================================
注意
================================================================================
无

================================================================================
