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
本样例展示I2S中断方式全双工主、从机通信功能。

说明：
本样例实现I2S主、从机全双工通信功能，使用内部时钟，音频采样频率位48K，通过两块板
子分别下载主机程序和从机程序，实现全双工通信。

================================================================================
测试环境
================================================================================
测试用板:
---------------------
EV_F460_LQ100_Rev2.0

辅助工具:
---------------------
杜邦线

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）使用杜邦线连接2块板子的通信引脚，引脚连接对应关系如下：
    目标板1                     目标板2
    PA7 <---------------------> PA7
    PB0 <---------------------> PB0
    PC4 <---------------------> PC5
    PC5 <---------------------> PC4
    GND <---------------------> GND
2）打开工程并设置宏I2S_UNIT_MASTER_SLAVE的值为I2S_MD_MASTER；重新编译程序，并将
程序下载到目标板1中；
3）修改宏I2S_UNIT_MASTER_SLAVE的值为I2S_MD_SLAVE；重新编译程序，并将程序下载到目
标板2中；
4）按下目标板1的K2按键，启动全双工数据循环收发，收发完一次数据后对接收到的数据与
发送的数据对比，完全相同则LED_BLUE亮，则功能正常；否则，LED_RED亮。

================================================================================
注意
================================================================================
无

================================================================================
