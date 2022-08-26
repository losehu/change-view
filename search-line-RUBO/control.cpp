/**
 //@FileName    :control.c
 //@CreatedDate :2021年12月22日
 //@Author      :LiHao
 //@Description :
 **/

#include "control.h"
#include <math.h>

CarInfoTypedef CarInfo;
//定时器中断 1ms
static int16 Timer_IntCnt = 0;
int16 SpeedOut_L = 0;
int16 SpeedOut_R = 0;
int16 DirectionOut = 0;
int16 ExSpeed = 250; //320 NormalSpeed
int16 ExSpeed_SUM = 250; //250
int16 ExSpeed_SUM_tmp=250;
int16 circle_speed = 270 ; //240
int16 fork_speed=240;
uint16_t Ackerman_kl = 750;    //左右差速系数
uint16_t Ackerman_kr = 750;
int16 ExSpeed_L = 0;
int16 ExSpeed_R = 0;
int16 L_SingleControlOut = 0, R_SingleControlOut = 0;
int8 InControlPeriod_Flag = 'F';

/**
 *@Name          :Timer6_IT
 *@Description   :Timer6_IT基本定时器6中断服务函数，用于电机控制及其他中断服务
 *@Param         :None
 *@Return        :NULL
 *@Sample        :Timer6_IT();
 **/
int cam_die = 0;
bool protect_cam = 0;
void Timer6_IT(void) {
    Timer_IntCnt++;
    CarInfo.UpTime += 0.001f;
    //  Beep_IntCnt();
    KeyScan();
//    //陀螺仪数值获取
////  Data_Filter();
////  if(TIMx_Cnt % 5 == 0 )
////  {
////      //角度积分
////      ICM_I_Int();
////      //更新角度
////      IMUupdate(ICM_Treated.gyro.x*0.01745, ICM_Treated.gyro.y*0.01745, ICM_Treated.gyro.z*0.01745, 。
////              ICM_Treated.accdata.x , ICM_Treated.accdata.y, ICM_Treated.accdata.z);
////  }
//    //-------------------------------方向控制------------------------------------//
//
    DirectionControl();
//
//    //-------------------------------速度控制------------------------------------//

    if (Timer_IntCnt % 10 == 0) {
        ExSpeed_L = ExSpeed_SUM;
        ExSpeed_R = ExSpeed_SUM;
        ExSpeedControl(); //顺序不能错
        DiffSpeedControl();
        SpeedControl();
    }


    //清零计数
    if (Timer_IntCnt >= 1000) {
        Timer_IntCnt = 0;
        CarProtect_IntCnt = 0;
    }
}
//初始化一些设置
uint8 CarSystem_Init(void) {
    //设置标志
    connect.error_cnt=0;
    connect.last_room=0;
    connect.room_cnt=0;
    CarInfo.Protect_Flag = 'F';
    CarInfo.ControlMode = CAMMODE;
    CarInfo.ClosedLoopMode = AllLoop;
    CarInfo.CAR_PROTECT_MODE = 'F'; //开启电磁信号削减保护
    CarInfo.StopTime = 0.000f;
    CarInfo.UpTime = 0.000f;
    CarInfo.ReadFlashParams = 'F';
    CarInfo.Camera_FPS = 150;
    //ret
    return 1;
}

//中线计算
int32 point_center;
int32 point_center0;
int32 point_center1;
int32 point_center2;
//uint32_t RunCnt = 0;
/**
 *@Name         :AllImageDeal
 *@Description  :AllImageDeal
 *@Param            :None
 *@Return       :NULL
 *@Sample       :AllImageDeal();
 **/

void AllImageDeal(void) {

    GetSimBinImage();
    search_line();
    judge_unit();
    solve_line();
    cal_middle_line();

}

/**
 *@Name         :ControlPeriodCheck
 *@Description  :ControlPeriodCheck控制周期检查:在一个舵机周期内完成方向控制
 *@Param            :None
 *@Return       :NULL
 *@Sample       :ControlPeriodCheck();
 **/
void ControlPeriodCheck(void) {

}

/**
 *@Name         :DirectionControl
 *@Description  :DirectionControl
 *@Param            :None
 *@Return       :None
 *@Sample       :DirectionControl();
 **/
void DirectionControl(void) {

}

/**
 *@Name         :ExSpeedControl
 *@Description  :ExSpeedControl这里用于加减速控制
 *@Param            :None
 *@Return       :NULL
 *@Sample       :ExSpeedControl();
 **/
void ExSpeedControl(void) {
//    if (1) {
//        ExSpeed = ExSpeed;
//    }
}

/**
 *@Name         :DiffSpeedControl
 *@Description  :DiffSpeedControl差速控制
 *@Param            :None
 *@Return       :NULL
 *@Sample       :DiffSpeedControl();
 **/
void DiffSpeedControl(void) {
    if (stop_flag == 1)
        ExSpeed = 0;
        //  else   ExSpeed=ExSpeed_SUM-(ROW-1-Base.Topline)/(ROW-1)*10 ;
    else
        ExSpeed = ExSpeed_SUM;

    //  Ackerman_kl=(1.0*Wheel_L.SpeedNow/   ExSpeed)*200;
    //     Ackerman_kr=(1.0*Wheel_R.SpeedNow/   ExSpeed)*200;
    Ackerman_kl = 180;
    Ackerman_kr = 200;
    if (CarInfo.ClosedLoopMode == AllLoop && CarInfo.Protect_Flag == 'F') {
        if (ServoAdd <= -300)    //右转
        {
            ExSpeed_L = (int) (ExSpeed
                               * (1
                                  - ((float) Ackerman_kr / 1000)
                                    * tan((float) ServoAdd * 3.14 / 4600)
                                    / 0.396));;
            ExSpeed_R = (int) (ExSpeed
                               * (1
                                  + ((float) Ackerman_kr / 1000)
                                    * tan((float) ServoAdd * 3.14 / 4600)
                                    / 0.396));
        } else if (ServoAdd > 300)    //左转
        {
            ExSpeed_L = (int) (ExSpeed
                               * (1
                                  - ((float) Ackerman_kl / 1000)
                                    * tan((float) ServoAdd * 3.14 / 4600)
                                    / 0.396));
            ExSpeed_R = (int) (ExSpeed
                               * (1
                                  + ((float) Ackerman_kl / 1000)
                                    * tan((float) ServoAdd * 3.14 / 4600)
                                    / 0.396));;
        } else {
            ExSpeed_L = ExSpeed;
            ExSpeed_R = ExSpeed;    //内轮减速外轮加速确保中轴线速度不变
        }
    }
}
/**
 *@Name         :SpeedControl
 *@Description  :SpeedControl速度PID控制
 *@Param            :None
 *@Return       :NULL
 *@Sample       :SpeedControl();
 **/
void SpeedControl(void) {


}
/**
 *@Name         :Timer7_IT
 *@Description  :Timer7_IT基本定时器7中断服务函数，用于图像处理及舵机控制
 *@Param            :None
 *@Return       :NULL
 *@Sample       :Timer7_IT();
 **/
void Timer7_IT(void) {
//    ControlPeriodCheck();
//    if (InControlPeriod_Flag == 'T') {//暂不使用
//       AllImageDeal();
//        DirectionControl();
//        InControlPeriod_Flag = 'F';
//    }
}

//冲出赛道电磁削减保护
int16 CarProtect_IntCnt = 0;
void CarProtect(void) {
    if ((CarInfo.UpTime >= 2.000f)
        && ((CarInfo.ClosedLoopMode == AllLoop)
            || (CarInfo.ClosedLoopMode == SpeedLoop))) {
        //---------------电磁保护--------------//
        if (ElcProtect_Flag == 'T' && CarInfo.CAR_PROTECT_MODE == 'T') {
            CarProtect_IntCnt++;
        }
        if (CarProtect_IntCnt >= 5) {
            CarInfo.Protect_Flag = 'T';
        }
    }
    if (CarInfo.StopTime != 0.000f && CarInfo.UpTime >= CarInfo.StopTime) {
        CarInfo.Protect_Flag = 'T';
        Base.element_check = 'F';
    }
}

//显示界面调整参数
const char CarInfoItem0[] = "Protect_flag En:";
const char CarInfoItem1[] = "Control Loop:";
const char CarInfoItem2[] = "Params Read:";


#define CarInfo_MENU_NUM 2
#define ControlMood_MENU_NUM 4

uint8 CarInfoShow(void) {

}

