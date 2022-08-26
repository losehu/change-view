/**
 //@FileName	:elc.c
 //@CreatedDate	:2021年12月22日
 //@Author		:LiHao
 //@Description	:
**/

#include "elc.h"

//statement
#define PROTECR_INDUCTOR_VAL  50
 uint8    ElcProtect_Flag;

//原始值
int32   LNow[ELC_NUM] = {0,0,0,0},
        LMax[ELC_NUM] = {0,0,0,0},
        LMin[ELC_NUM] = {0,0,0,0};
uint16  ELC[ELC_GROUP][ELC_TIME][ELC_NUM];
float   LNor[ELC_NUM];
uint16  LNORMAX[ELC_NUM] = {4095,4095,4095,4095};
uint8   MaxNorIndex = 0;
uint8   MinNorIndex = 0;
uint16  ElcSum_14 = 0;
uint16  ElcSum_23 = 0;
uint16  ElcSum_1234 = 0;
int16   ElcCenter_14 = 0;
int16   ElcCenter_23 = 0;
uint8   ElcPROTECT_FLAG;
uint16  KFP_ElcSum_14 = 0;
uint16  KFP_ElcSum_23 = 0;
float  ElcCenter_14_100;

//读取电感值
void ElcRead(void)
{

}


//归一化
void ElcNormalized(void)
{

}


//中线误差计算 使用第一次归一化的数值
//使用差比和算法
#define ElcMAX_CENTER_ERROR 100
void ElcCenterCalculate(void)
{

}


//电磁标志设置
void ElcStatus_Init(void)
{
    //Circle
    ElcCircle.FindFlag = 'F';
    ElcCircle.NowValLessThanMaxCnt = 0;
    ElcCircle.SUM_MAX = 0;
    ElcCircle.CircleInTurnFlag = 'F';
}


//电感检测元素 环岛 十字
uint8 ElcJudge(void)
{

}


//---------------------------------------------CIRCLE-----------------------------------------//
ElcCircleTypedef ElcCircle;


//电磁联合摄像头联合判断环岛拐点
uint8 ElcCircleJudge(void)
{
//  if(ABS(ElcCenter_23) < 20 && ElcSum_14 >= 125)
//  {
//      Beep_DiDi(1,100);
//  }
//  //发现标志
//  if(ElcSum_14 > 50 && ElcCircle.FindFlag == 'F' && Circle.FindFlag == 'T')
//  {
//      ElcCircle.FindFlag = 'T';
//  }
//  //计算最大值
//  if(ElcSum_14 > ElcCircle.SUM_MAX && ElcCircle.FindFlag == 'T')
//  {
//      ElcCircle.SUM_MAX = ElcSum_14;
//      ElcCircle.NowValLessThanMaxCnt = 0;
//  }
//  if(ElcSum_14 < (ElcCircle.SUM_MAX - 3) && ElcCircle.FindFlag == 'T')
//  {
//      ElcCircle.NowValLessThanMaxCnt ++;
//  }
//  if(ElcCircle.NowValLessThanMaxCnt >= 10 && ElcCircle.CircleInTurnFlag == 'F')
//  {
//      ElcCircle.CircleInTurnFlag = 'T';
//      Beep_DiDi(1,100);
//  }
//  if(ElcCircle.CircleInTurnFlag == 'T' && Circle.FindFlag == 'F')
//  {
//      ElcCircle.FindFlag = 'F';
//      ElcCircle.CircleInTurnFlag = 'F';
//      ElcCircle.NowValLessThanMaxCnt = 0;
//      ElcCircle.SUM_MAX = 0;
//  }
    return 1;
}



//LCD显示 第5页
uint8 ElcPage(void)
{
    while(KeyLast != onepress)
    {
        ElcRead();
        ElcNormalized();
        ElcCenterCalculate();
        ElcShow();

    }
    KeyLast = nopress;
    return 0;
}


//电感的显示界面
uint8 ElcShow(void)
{

}
