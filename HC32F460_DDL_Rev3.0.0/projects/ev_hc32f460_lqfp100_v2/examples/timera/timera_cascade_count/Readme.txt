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
本例程用TimerA的单元1和单元2，用级联计数方式实现10毫秒定时，并将定时时序用PA2输出。

说明：
1. TimerA的计数位宽为16位，用对称单元的级联计数方式，可实现32位计数；
2. 对称单元的对应关系：单元1和单元2，单元3和单元4，单元5和单元6；
3. 对称单元用作级联计数时，其中一个设置为低位计数单元，另一个设置为高位计数单元；
   其中，高位计数单元的时钟为低位计数单元的溢出事件（上溢或下溢，当计数值达到周期
   值时，产生溢出）；
4. 例程中，低位计数单元的定时周期为1毫秒；

================================================================================
测试环境
================================================================================
测试用板:
---------------------
EV_F460_LQ100_Rev2.0

辅助工具:
---------------------
示波器，杜邦线

辅助软件:
---------------------

================================================================================
使用步骤
================================================================================
1）将示波器探头连接至PA2；
2）打开工程，重新编译，启动调试或直接下载程序运行；
3）运行程序，可看到示波器显示一路频率为 50Hz 的方波。

================================================================================
注意
================================================================================
无

================================================================================
