该工程是RM2019机甲大师比赛外赠步兵机器人的嵌入式程序，使用的芯片型号为stm32f427IIH6，使用st公司标准库。

The project is the program for the embedded system for the Robomaster 2019 competition. The chip is stm32f427IIH6 that uses the standard library provided by STMicroelectronics.


目录：Table of Contents

CMSIS：内核相关的文件 Files related to the kernel

FWLIB：标准库文件 Standard library files

Project：工程文件 Project files

startup:芯片启动文件 Loading files when the chip boots

user：用户编写的相关文件，主要编写的文件都在这个文件夹下 Files written by developers. This should be the place the majority of your code.

user/main.c\h :main函数所在的文件 The file that contains the main function

user/AHRS：陀螺仪驱动以及姿态解算 Gyroscope driver and posture computation

user/APP：freeRTOS任务 freeRTOS tasks

user/DSP：DSP库 DSP libraries

user/FreeRTOS:移植的freeRTOS文件 Ported freeRTOS files

user/hardware：硬件层 Hardware layer

user/user_lib：编写的数学函数 Mathematical functions
