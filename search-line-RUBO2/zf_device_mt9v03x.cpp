/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file             zf_device_mt9v03x
* @company          成都逐飞科技有限公司
* @author           逐飞科技(QQ790875685)
* @version          查看doc内version文件 版本说明
* @Software         MounRiver Studio V1.51
* @Target core      CH32V307VCT6
* @Taobao           https://seekfree.taobao.com/
* @date             2021-11-25
* @note             version:
*                   V1.1 2021.12.23 摄像头采集完成标志位增加volatile修饰
*
*                   接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   TXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX_DVP        宏定义
*                   RXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX_DVP        宏定义
*                   D0                  查看 zf_device_mt9v03x.h 中 MT9V03X_D0_PIN_DVP             宏定义
*                   D1                  查看 zf_device_mt9v03x.h 中 MT9V03X_D1_PIN_DVP             宏定义
*                   D2                  查看 zf_device_mt9v03x.h 中 MT9V03X_D2_PIN_DVP             宏定义
*                   D3                  查看 zf_device_mt9v03x.h 中 MT9V03X_D3_PIN_DVP             宏定义
*                   D4                  查看 zf_device_mt9v03x.h 中 MT9V03X_D4_PIN_DVP             宏定义
*                   D5                  查看 zf_device_mt9v03x.h 中 MT9V03X_D5_PIN_DVP             宏定义
*                   D6                  查看 zf_device_mt9v03x.h 中 MT9V03X_D6_PIN_DVP             宏定义
*                   D7                  查看 zf_device_mt9v03x.h 中 MT9V03X_D7_PIN_DVP             宏定义
*                   PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN_DVP           宏定义
*                   VSYNC               查看 zf_device_mt9v03x.h 中 MT9V03X_VSY_PIN_DVP            宏定义
*                   HSYNC               查看 zf_device_mt9v03x.h 中 MT9V03X_HERF_PIN_DVP           宏定义
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_device_mt9v03x.h"



uint8_t *camera_buffer_addr;                                                    // 摄像头缓冲区地址指针

uint8_t volatile mt9v03x_finish_flag_dvp = 0;                                                // 一场图像采集完成标志位
 unsigned char **mt9v03x_image_dvp;

static          uint8_t     receive_dvp[3];
static          uint8_t     receive_num_dvp = 0;
static volatile uint8_t     uart_receive_flag_dvp;

