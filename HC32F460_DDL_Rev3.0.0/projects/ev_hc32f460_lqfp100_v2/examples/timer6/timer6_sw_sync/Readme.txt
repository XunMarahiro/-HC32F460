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
本样例主要展示MCU的TIMER6的3组定时器同步启动（输出PWM）、同步停止、同步清零功能
说明：
配置timer6，hclk设置为200M
所有单元的CHA的比较寄存器都使能单缓存功能,所有单元的CHB不使能buffer功能
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
1）示波器连接观察PWM输出
            PWMA         PWMB
   Timer6_1：PA8         PE8
   Timer6_2：PE11        PE10
   Timer6_3：PE13        PB15
2）打开工程编译并全速运行。
3）使用示波器查看上述6个引脚的信号，可以看到6路PWM同步开始和停止。
   所有单元CHA输出的PWM占空比交替变化，所有单元CHB输出的PWM占空比不变。

================================================================================
注意
================================================================================
无

================================================================================
