//
// Created by Administrator on 2022/3/14.
//

#ifndef MAIN_CPP_ZF_COMMON_HEADFILE_H
#define MAIN_CPP_ZF_COMMON_HEADFILE_H

#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
#define USE_ZF_TYPEDEF      1                                                   // 是否启用类型定义申明
#if USE_ZF_TYPEDEF
// 数据类型声明
// 尽量使用 stdint.h 定义的类型名称 避免冲突 这里可以裁剪
typedef unsigned char       uint8;                                              // 无符号  8 bits
typedef unsigned short int  uint16;                                             // 无符号 16 bits
typedef unsigned long int   uint32;                                             // 无符号 32 bits
typedef unsigned long long  uint64;                                             // 无符号 64 bits

typedef char                int8;                                               // 有符号  8 bits
typedef short int           int16;                                              // 有符号 16 bits
typedef long  int           int32;                                              // 有符号 32 bits
typedef long  long          int64;                                              // 有符号 64 bits

typedef volatile uint8      vuint8;                                             // 易变性修饰 无符号  8 bits
typedef volatile uint16     vuint16;                                            // 易变性修饰 无符号 16 bits
typedef volatile uint32     vuint32;                                            // 易变性修饰 无符号 32 bits
typedef volatile uint64     vuint64;                                            // 易变性修饰 无符号 64 bits

typedef volatile int8       vint8;                                              // 易变性修饰 有符号  8 bits
typedef volatile int16      vint16;                                             // 易变性修饰 有符号 16 bits
typedef volatile int32      vint32;                                             // 易变性修饰 有符号 32 bits
typedef volatile int64      vint64;                                             // 易变性修饰 有符号 64 bits
#endif

#define         TRFED_ROW               188
#define         TRFED_COL               120

#include "zf_device_mt9v03x.h"
#include "zf_device_icm20602.h"
#include "All_Init.h"
#include "image_deal.h"
#include "cam.h"
#include "element.h"
#include "ty.h"
#include "control.h"
#include "Device.h"
#include "elc.h"
#include "ty.h"
extern Mat src;
#define     myabs(x)            ((x)>=0? (x): -(x))
#define     limit_ab(x,a,b)     (((x)<(a))?(a):(((x)>(b))?(b):(x)))
#define     max_ab(a,b)         (((a)>=(b))? (a):(b))
#define     min_ab(a,b)         (((a)>=(b))? (b):(a))

#define WHITE           4  // 白色
#define BLACK           3  // 黑色
#define BLUE            2  // 蓝色
#define PURPLE          0XF81F  // 紫色
#define PINK            0XFE19  // 粉色
#define RED             1  // 红色
#define MAGENTA         0xF81F  // 品红
#define GREEN           5  // 绿色
#define CYAN            0x07FF  // 青色
#define YELLOW          6  // 黄色
#define BROWN           7  // 棕色
#define GRAY            0X8430  // 灰色

#endif //MAIN_CPP_ZF_COMMON_HEADFILE_H
