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
本样例主要展示CMP的单一比较功能。
说明：
无

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
1）打开工程并重新编译;
2）连接PA4(CMP2_INP1)与PC0(ADC电位器输出)
   示波器连接PB13(VCOUT2)查看输出；
3）全速运行样例代码;
4）调节ADC电位器改变比较电压值，查看LED灯变化和PB13的VCOUT2;
5）当比较电压<参考电压，蓝色LED亮，VCOUT2输出低;
   当比较电压>参考电压，红色LED亮，VCOUT2输出高;
   本样例参考电压为VCC/2。

================================================================================
注意
================================================================================


================================================================================
