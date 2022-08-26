/**
 //@FileName	:Device.c
 //@CreatedDate	:2021年12月19日
 //@Author		:LiHao
 //@Description	:各种外设和设备初始化和测试
**/

#include "Device.h"


/**
*@Name			:Devices_Init
*@Description 	:外设接口初始化
*@Param			:None
*@Return		:None
*@Sample        :Devices_Init();
**/
void Devices_Init(void)
{
    //LED

}

//==============================================================//
//=========================外设服务函数==========================//
//==============================================================//

//=============================LED==============================//
/**
*@Name			:LEDTest
*@Description 	:LEDTest
*@Param			:None
*@Return		:None
*@Sample		:LED_Test();
**/
void LEDTest(void)
{

}

//============================BEEP==============================//
BeepTypedef Beep;
MY_BEEP STATE_BEEP;
void BEEP_RING(int time)
{
    STATE_BEEP.flag=1;
    STATE_BEEP.ms=time;
}
/**
*@Name			:BeepTest
*@Description 	:BeepTest
*@Param			:None
*@Return		:None
*@Sample		:Beep_Test();
**/
void BeepTest(void)
{

}

/**
*@Name			:BeepTick
*@Description 	:BeepTick,中断里计时
*@Param			:n：响几次；time：间隔
*@Return		:None
*@Sample		:BeepTick(2, 100);
**/
void BeepTick(uint8_t n, uint16_t time)
{

}

/**
*@Name			:Beep_IntCnt
*@Description 	:Beep_IntCnt蜂鸣器中断
*@Param			:
*@Return		:
*@Sample		:
**/
void Beep_IntCnt(void)
{

}

/**
*@Name			:BeepParams_Init
*@Description 	:BeepParams_Init
*@Param			:
*@Return		:
*@Sample		:
**/
void BeepParams_Init(void)
{
    Beep.State = 'F';
    Beep.Cnt = 0;
    Beep.Num = 0;
    Beep.Time = 0;
    STATE_BEEP.flag=0;
    STATE_BEEP.ms=0;
}
//=============================KEY==============================//

KeyStateEnum KeyUp, KeyDown, KeyLeft, KeyRight, KeyCenter, KeyNext, KeyLast;
/**
*@Name			:KeyScan
*@Description 	:KeyScan//在中断里扫描
*@Param			:None
*@Return		:None
*@Sample		:KeyScan();
**/
void KeyScan(void)
{

}

/**
*@Name			:KeyParams_Init
*@Description 	:KeyParams_Init
*@Param			:None
*@Return		:None
*@Sample		:KeyParams_Init();
**/
void KeyParams_Init(void)
{
    KeyUp = nopress;
    KeyDown = nopress;
    KeyCenter = nopress;
    KeyLeft = nopress;
    KeyRight = nopress;
    KeyNext = nopress;
    KeyLast = nopress;
}
//============================CODE==============================//
/**
*@Name			:CodeScan
*@Description 	:CodeScan拨码开关
*@Param			:None
*@Return		:None
*@Sample		:CodeScan();
**/
void CodeScan(void)
{

}

//============================ENCODER==============================//
WheelTypedef   Wheel, Wheel_L, Wheel_R;
int32_t Encoder_L_Cnt = 0;
int32_t Encoder_R_Cnt = 0;
int32_t EncoderAll_Cnt = 0;
/**
*@Name			:EncoderRead
*@Description 	:EncoderRead
*@Param			:None
*@Return		:None
*@Sample		:EncoderRead();
**/
void EncoderRead(void)
{

}
/**
*@Name			:EncoderShow
*@Description 	:EncoderShow
*@Param			:None
*@Return		:1
*@Sample		:EncoderShow();
**/
uint8 EncoderShow(void)
{

}

/**
*@Name			:EncoderParams_Init
*@Description 	:EncoderParams_Init
*@Param			:
*@Return		:
*@Sample		:EncoderParams_Init();
**/
void EncoderParams_Init(void)
{
    Wheel.SpeedNow = 0;
    Wheel.SpeedLast = 0;
    Wheel.Distance = 0;
    Wheel.Acc = 0;

    Wheel_L.SpeedNow = 0;
    Wheel_L.SpeedLast = 0;
    Wheel_L.Distance = 0;
    Wheel_L.Acc = 0;

    Wheel_R.SpeedNow = 0;
    Wheel_R.SpeedLast = 0;
    Wheel_R.Distance = 0;
    Wheel_R.Acc = 0;
}
//=============================MOTOR==============================//
short pwm_l = 0;
short pwm_r = 0;
/**
*@Name			:MotorWrite
*@Description 	:MotorWrite
*@Param			:None
*@Return		:None
*@Sample		:MotorWrite();
**/
int last_speed=0;
void MotorWrite(void)
{

}


#define MOTOR_MENU_NUM  1
/**
*@Name			:MotorShow
*@Description 	:MotorShow
*@Param			:None
*@Return		:None
*@Sample		:MotorShow();
**/
void MotorShow(void)
{

}
//=============================SERVO==============================//
short ServoAdd = 0;
/**
*@Name			:ServoWrite
*@Description 	:ServoWrite
*@Param			:None
*@Return		:None
*@Sample		:ServoWrite();
**/
void ServoWrite(void)
{

}
/**
*@Name			:ServoShow
*@Description 	:ServoShow
*@Param			:None
*@Return		:None
*@Sample		:ServoShow();
**/
void ServoShow(void)
{

}

/**
*@Name          :DevicesParams_Init
*@Description   :DevicesParams_Init 外设参数初始化
*@Param         :
*@Return        :
*@Sample        :
**/
void DevicesParams_Init(void)
{
    BeepParams_Init();
    KeyParams_Init();
    EncoderParams_Init();
}



