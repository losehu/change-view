/**
 //@FileName    :image_deal.c
 //@CreatedDate :2021年12月31日
 //@Author      :LiHao&WuSheng
 //@Description :图像的基本处理，剪切|二值化|逆透视
 **/

#include "image_deal.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
//int ipts0[POINTS_MAX_LEN][2];
//int ipts1[POINTS_MAX_LEN][2];
//int ipts0_num, ipts1_num;
//
//float thres = 20;
//float block_size = 7;
//float clip_value = 2;
//float begin_x = 5;
//float begin_y = 90;
//uint8_t CamImage[USED_ROW][USED_COL];//总钻风获取的灰度图像，由ImageBuffer()获取
uint8_t SimBinImage[USED_ROW][USED_COL]; //简易无逆透视二值化图像，由GetSimBinImage()获取
uint8_t AdpBinImage[USED_ROW][USED_COL]; //自适应阈值二值化图像，
//uint8_t PerImage[TRFED_ROW][TRFED_COL];//Transformed_Binary_Image透视变换后的图像

int16_t ThresholdAdd = 0; //对大津法得到的阈值进行手动调节
uint8_t *PerImg_ip[TRFED_ROW][TRFED_COL];
int16 seed_left_x, seed_left_y, seed_right_x, seed_right_y;
/**
 *@Name         :ImageBuffer
 *@Description  :ImageBuffer图像缓冲&剪切到CamImage中
 *@Param            :None
 *@Return       :None
 *@Sample       :ImageBuffer();
 **/
int fake_do = 0;
void ImageBuffer(void) {
    //memcpy(&CamImage, &mt9v03x_image_dvp, USED_ROW*USED_COL);
    uint8_t FirstRow = 0; //起始行 从0开始
    uint8_t FirstCol = (MT9V03X_DVP_W - USED_COL) / 2; //起始列
    if (FirstRow > MT9V03X_DVP_H - USED_ROW) {
        FirstRow = MT9V03X_DVP_H - USED_ROW;
    }
    for (uint8_t i = 0; i < USED_ROW; i++) {
        memcpy(&CamImage[i][0], &mt9v03x_image_dvp[FirstRow][FirstCol],
               USED_COL);
        FirstRow++;
    }
}
/**
 *@Name         :GetBinImage
 *@Description  :GetBinImage
 *@Param            :img[ROW][COL]  原图像
 *@Return       :None
 *@Sample       :GetSimBinImage(CamImage[ROW][COL]);
 **/
void GetSimBinImage(void) {
}
/**
 *@Name         :GetPerImage
 *@Description  :GetPerImage 得到逆透视后的图像，用于显示
 *@Param            :None
 *@Return       :None
 *@Sample       :GetPerImage();
 **/
//void ShowPerImage(void)
//{
//    uint8_t PerImage[TRFED_ROW][TRFED_COL];
//    uint16_t i = 0;
//    uint16_t j = 0;
//    for (i = 0; i < TRFED_ROW; i++) {
//        for (j = 0; j < TRFED_COL; j++) {
//            PerImage[i][j] = *PerImg_ip[i][j];
//        }
//    }
//    ips114_displayimage032(PerImage[0],TRFED_COL,TRFED_ROW);
//    //ips114_show_gray_image_vec(0,0,PerImg_ip[0],TRFED_COL,TRFED_ROW,TRFED_COL,TRFED_ROW,0);
//}
/**
 *@Name         :ImagePerspectiveInit
 *@Description  :ImagePerspectiveInit 得到透视变换的图像的地址映射;只运行一次
 *@Param            :None
 *@Return       :None
 *@Sample       :ImagePerspectiveInit();
 **/
void ImagePerspective_Init(void) {
    static uint8_t BlackColor = GrayPoint;
    double change_un_Mat[3][3] = {          //114w*100h
            {-0.01609759704190238,  0.01932561893613478,   -2.040617594981866},
            {
             0.0004352209945470896, -0.000367865364438621,
                                                           -0.7035606436969671},
            {1.115951268069474e-005,
                                    0.0001970185393508392, -0.03104642853440032},};
    for (int i = 0; i < TRFED_COL; i++) {
        for (int j = 0; j < TRFED_ROW; j++) {
            int local_x = (int) ((change_un_Mat[0][0] * i
                                  + change_un_Mat[0][1] * j + change_un_Mat[0][2])
                                 / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
                                    + change_un_Mat[2][2]));
            int local_y = (int) ((change_un_Mat[1][0] * i
                                  + change_un_Mat[1][1] * j + change_un_Mat[1][2])
                                 / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
                                    + change_un_Mat[2][2]));
            if (local_x
                >= 0 && local_y >= 0 && local_y < USED_ROW && local_x < USED_COL)
                PerImg_ip[j][i] = &PER_IMG[local_y][local_x];
            else {
                PerImg_ip[j][i] = &BlackColor;          //&PER_IMG[0][0];
            }
        }
    }
}
/**
 *@Name         :GetAdpBinImage
 *@Description  :Get_Adaptive_Binary_Image
 *@Param            :block0 比较赋值区块大小必须为偶数；block1 计算阈值区块大小必须为偶数必须大于block0
 *@Return       :None
 *@Sample       :GetAdpBinImage(7);
 **/
void GetAdpBinImage(short block0, short block1) {
//    short block0 = 2;//内区块
//    short block1 = 4;//外区块
    uint8_t dx0 = 0;
    uint8_t dy0 = 0;
    uint8_t dx1 = 0;
    uint8_t dy1 = 0;
    uint8_t x = 0;
    uint8_t y = 0;
    uint16_t thres = 0;
    for (x = 0; x < USED_COL - (block1 - block0); x += block0) {
        for (y = 0; y < USED_ROW - (block1 - block0); y += block0) {
            thres = 0;
            for (dx1 = 0; dx1 < block1; dx1++) {          //计算block1里的阈值
                for (dy1 = 0; dy1 < block1; dy1++) {
                    thres += CamImage[y + dy1][x + dx1];
                }
            }
            thres = thres / (block1 * block1);
            for (dx0 = 0; dx0 < block0; dx0++) {          //对block0里的值二值化
                for (dy0 = 0; dy0 < block0; dy0++) {
                    AdpBinImage[y + dy0 + (block1 - block0) / 2][x + dx0
                                                                 + (block1 - block0) / 2] =
                            CamImage[y + dy0 + (block1 - block0) / 2][x + dx0
                                                                      + (block1 - block0) / 2] > thres ? 255 : 0;
                }
            }
            if (x == 0) {          //修左黑边
                for (uint8_t jx0 = 0; jx0 < (block1 - block0) / 2; jx0++) {
                    for (int ix0 = 0; ix0 < block0; ix0++) {
                        AdpBinImage[ix0][jx0] =
                                CamImage[ix0][jx0] > thres ? 255 : 0;
                    }
                }
            }
            if (x) {          //修右黑边

            }
            if (y == 0) {          //修上黑边
                for (uint8_t jy0 = 0; jy0 < block0; jy0++) {
                    for (int iy0 = 0; iy0 < (block1 - block0) / 2; iy0++) {
                        AdpBinImage[iy0][jy0] =
                                CamImage[iy0][jy0] > thres ? 255 : 0;
                    }
                }
            }
            if (y) {          //修下黑边

            }
        }
    }
}
/**
 *@Name         :OSTUThreshold
 *@Description  :OSTUThreshold大津法得到阈值
 *@Param            :img[USED_ROW][USED_COL]  原图像
 *@Return       :得到的阈值
 *@Sample       :由GetSimBinImage()调用
 **/
uint8_t OSTUThreshold(uint8_t img[USED_ROW][USED_COL]) {
    signed short j, i;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold0 = 0;
    unsigned char HistoGram[256];              //

    for (i = 0; i < 256; i++)
        HistoGram[i] = 0; //初始化灰度直方图

    for (i = 0; i < USED_ROW; i++) {
        for (j = 0; j < USED_COL; j++) {
            HistoGram[img[i][j]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0;
         MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (i = MinValue; i <= MaxValue; i++)
        Amount += HistoGram[i];        //  像素总数

    Pixelshortegral = 0;
    for (i = MinValue; i <= MaxValue; i++) {
        Pixelshortegral += HistoGram[i] * i;        //灰度值总数
    }
    SigmaB = -1;
    for (i = MinValue; i < MaxValue; i++) {
        PixelBack = PixelBack + HistoGram[i];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[i] * i;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore)
                * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold0 = i;
        }
    }
    return Threshold0;                        //返回最佳阈值;
}
///************************八领域*******************************/
size_point stack_seed[stack_size];    //栈
size_point stack_seed1[stack_size];    //栈
int16 start_line = 10;
int16 left_lost_cnt = 0, right_lost_cnt = 0;
int16 stack_top = 0;
bool stop_flag = 0;
int16 stack_top1 = 0;
int16 max_py = 0, max_py1 = 0;
int16 min_py = 0, min_py1 = 0;
const int CenterCalMinRow_tmp = 0;
const int CenterCalMaxRow_tmp = 40;
int CenterCalMinRow = CenterCalMinRow_tmp;
int CenterCalMaxRow = CenterCalMaxRow_tmp; //15
int CenterCalMinRow_circle = 0;
int CenterCalMaxRow_circle = 20; //15

int lose_error = 0;
#define line_debug 1
bool search_line_flag = 0;
//size_point connects[16] = {    //八领域扫点
//        {0,  -1},    //上
//        {-1, -1},    //左上123 567
//        {-1, 0},    //左
//        {-1, 1},    //左下
//        {0,  1},    //下
//        {1,  1},    //右下
//        {1,  0},    //右
//        {1,  -1},    //右上
//};
size_point connects[16] = {    //八领域扫点
        {0,  -1},    //上
        {-1, -1},    //左上123 567
        {-1, 0},    //左
        {-1, 1},    //左下
        {0,  1},    //下
        {1,  1},    //右下
        {1,  0},    //右
        {1,  -1},    //右上
};
int16 stack_max = 0;
bool pull_stack(int16 x, int16 y, uint8_t img_tmp[][COL]) //入栈
{
    if (stack_top == stack_size) {
        search_line_flag = 1;
        return 1;
    }
    img_tmp[y][x] = 1;
    stack_seed[stack_top].x0 = x;
    stack_seed[stack_top].y0 = y;
    stack_top++;
    return 0;
}
size_point push_stack() //出栈
{
    stack_seed[stack_top].y0 = 0;
    stack_seed[stack_top].x0 = 0;
    return stack_seed[--stack_top];
}
bool pull_stack1(int16 x, int16 y, uint8_t p_Pixels[][COL]) //入栈
{
    if (stack_top1 == stack_size) {
        search_line_flag = 1;
        return 1;
    }
    p_Pixels[y][x] = 1;
    stack_seed1[stack_top1].x0 = x;
    stack_seed1[stack_top1].y0 = y;
    stack_top1++;
    return 0;
}
size_point push_stack1() //出栈
{
    stack_seed1[stack_top1].y0 = 0;
    stack_seed1[stack_top1].x0 = 0;
    return stack_seed1[--stack_top1];
}
int start = 0;
float turn_num[2][ROW / 2];
int cnt_way[9];
int16 up_left_flag, up_right_flag, up_left_num, up_right_num;
bool judge_diff(int center_row, int center_col, uint8_t img_tmp[ROW][COL]) {
    for (int i = center_row - 1; i <= center_row + 1; i++) {
        if (i < 0 || i >= ROW)continue;
        for (int j = center_col - 1; j <= center_col + 1; j++) {
            if (j < 0 || j >= COL || (i == center_row && j == center_col))continue;
            if (!img_tmp[i][j] && ImageUsed[ROW - 1 - i][j] == 0)return 1;
        }
    }
    return 0;
}
void cal_middle_line(void) {
//计算当前行的偏移量
//    float excursion[ROW];
    float sumcenter = 0;
    float cnt = 0;
    for (Ysite = CenterCalMinRow; Ysite < CenterCalMaxRow; Ysite++) {
        if (Ysite == stack_top || Ysite == stack_top1)
            continue;
        if (Img.LeftBorderFindFlag[Ysite] == 'T'
            || Img.RightBorderFindFlag[Ysite] == 'T') {
            sumcenter += (float) (Img.Center[Ysite] - (COL / 2))
                         / (float) (RoadWide0 / 2) * 100;
            cnt++;
        }
    }
    point_center0 = (int32) (sumcenter / (cnt) * 2.0f);
    point_center = point_center0 * 0.6f + point_center1 * 0.3f
                   + point_center2 * 0.1f;
    point_center2 = point_center1;
    point_center1 = point_center0;
    if (circle_flag.enter_left_circle == 1 && circle_flag.on_left_circle == 0) {
        if (point_center0 > -point_center1)
            point_center0 = point_center0 / 2;
    }
}

void solve_line() {

    /*****************正常************************/
    bool error_right_flag = 0, error_left_flag = 0;
    int error_right_tmp = 0, error_left_tmp = 0;
    if (circle_flag.find_left_circle == 0 && circle_flag.find_right_circle == 0
        && fork_flag.find_fork == 0 && Cross.FindFlag == 'F'
        && door_flag.find_door == 0) {
        for (int Ysite = StartScanRow; Ysite < ROW; Ysite++) {
            if (((Img.RightBorderFindFlag[Ysite] == 'T'
                  && Img.LeftBorderFindFlag[Ysite] == 'F')
                 || (Img.RightBorderFindFlag[Ysite] == 'F'
                     && Img.LeftBorderFindFlag[Ysite] == 'T'))
                && (Img.RightBorder[Ysite] - Img.LeftBorder[Ysite]
                    >= RoadWide0 * 2) && Img.LeftBorder[Ysite] < COL / 4 && Img.RightBorder[Ysite] > COL * 0.75) {
                Img.LeftBorderFindFlag[Ysite] = 'F';
                Img.RightBorderFindFlag[Ysite] = 'F';
            }
            if (Ysite < StartScanRow + 1) {
                if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    Img.Center[Ysite] = (Img.LeftBorder[Ysite] + Img.RightBorder[Ysite]) / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
                    Img.Center[Ysite] = CenterCol;
                else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
                    Img.Center[Ysite] = Img.LeftBorder[Ysite] + Img.RoadWide[Ysite] / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
                    Img.Center[Ysite] = Img.RightBorder[Ysite] - Img.RoadWide[Ysite] / 2;



                //  Img.LeftBorderFindFlag[Ysite]='T',Img.RightBorderFindFlag[Ysite]='T';
            } else {
//                if (((Img.RightBorderFindFlag[Ysite] == 'T'
//                      && Img.LeftBorderFindFlag[Ysite] == 'F')
//                     || (Img.RightBorderFindFlag[Ysite] == 'F'
//                         && Img.LeftBorderFindFlag[Ysite] == 'T'))
//                    && (Img.RightBorder[Ysite] - Img.LeftBorder[Ysite]
//                        >= RoadWide0 * 2)&&(Img.LeftBorder[Ysite]<COL/10&&Img.RightBorder[Ysite]>COL*0.9)) {
//                    Img.LeftBorderFindFlag[Ysite] = 'F';
//                    Img.RightBorderFindFlag[Ysite] = 'F';
//                }
                //此处应对有丢边界的中线进行处理
                //边界直接平移
                //                       if(((Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'F')||
                //                              ( Img.RightBorderFindFlag[Ysite] == 'F'&& Img.LeftBorderFindFlag[Ysite] == 'T'))&&
                //                                                (  Img.RightBorder[Ysite]-Img.LeftBorder[Ysite]>=RoadWide0*2+5)
                //                               )
                //                       {
                //                           Img.LeftBorderFindFlag[Ysite]='F';
                //                           Img.RightBorderFindFlag[Ysite]='F';
                //                       }



                if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    Img.Center[Ysite] = (Img.LeftBorder[Ysite] + Img.RightBorder[Ysite]) / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
                    Img.Center[Ysite] = CenterCol;
                else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
                    Img.Center[Ysite] = Img.LeftBorder[Ysite] + Img.RoadWide[Ysite] / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
                    Img.Center[Ysite] = Img.RightBorder[Ysite] - Img.RoadWide[Ysite] / 2;

                //RUBO_FIX






                //边界按斜率平移
                //            if(Img.RightBorderFindFlag[Ysite]=='T' && Img.LeftBorderFindFlag[Ysite]=='T' && .
                //                    (myabs(Img.LeftBorderDy[Ysite]))<=15 && (myabs(Img.RightBorderDy[Ysite])<=15))
                //                Img.Center[Ysite] = (Img.LeftBorder[Ysite] + Img.RightBorder[Ysite]) / 2;
                //            else if(Img.LeftBorderFindFlag[Ysite]=='T' && Img.RightBorderFindFlag[Ysite]=='F' && .
                //                    (myabs(Img.LeftBorderDy[Ysite]))<=15)//仅有左边界
                //                Img.Center[Ysite] = Img.LeftBorder[Ysite] + CenterCalAry[myabs(Img.LeftBorderDy[Ysite])];
                //            else if(Img.LeftBorderFindFlag[Ysite]=='F' && Img.RightBorderFindFlag[Ysite]=='T' && .
                //                    (myabs(Img.RightBorderDy[Ysite])<=15))//仅有右边界
                //                Img.Center[Ysite] = Img.RightBorder[Ysite] - CenterCalAry[myabs(Img.RightBorderDy[Ysite])];
                //            else if(Img.LeftBorderFindFlag[Ysite]=='F' && Img.RightBorderFindFlag[Ysite]=='F')
                //                Img.Center[Ysite] = CenterCol;
                //            else
                //                Img.Center[Ysite] = (Img.LeftBorder[Ysite] + Img.RightBorder[Ysite]) / 2;
                //            if(Ysite >= 2)//滤波限幅
                //                Img.Center[Ysite] = 0.6 * Img.Center[Ysite] + 0.3 * Img.Center[Ysite - 1] + 0.1 * Img.Center[Ysite - 2];
                // Img.Center[Ysite] = limit_ab(Img.Center[Ysite], 0, COL - 1);
            }

//            //此处应对有丢边界的中线进行处理
//            if (Img.RightBorderFindFlag[Ysite] == 'T'
//                    && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
//                            {
//                Img.Center[Ysite] = (Img.LeftBorder[Ysite]
//                        + Img.RightBorder[Ysite]) / 2;
//            } else if (Img.RightBorderFindFlag[Ysite] == 'F'
//                    && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
//                Img.Center[Ysite] = CenterCol;
//            else if (Img.RightBorderFindFlag[Ysite] == 'F'
//                    && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
//                            {
//                Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 * 0.4;
//            } else if (Img.RightBorderFindFlag[Ysite] == 'T'
//                    && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
//                            {
//                Img.Center[Ysite] = Img.RightBorder[Ysite] - RoadWide0 * 0.4;
//            }
        }
        return;
    }
    /*****************环岛************************/
    if (circle_flag.find_left_circle == 1) {
        if (circle_flag.enter_left_circle == 0) {
            for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                //此处应对有丢边界的中线进行处理
                if (Img.RightBorderFindFlag[Ysite] == 'T'
                    && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线

                    Img.Center[Ysite] = Img.RightBorder[Ysite] - RoadWide0 / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'T'
                         && Img.LeftBorderFindFlag[Ysite] == 'F') //丢了左边线

                    Img.Center[Ysite] = Img.RightBorder[Ysite] - RoadWide0 / 2;
                else
                    Img.Center[Ysite] = COL / 2;
            }
        } else {
            if (circle_flag.on_left_circle == 0) {
                for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                    Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 * 0.7; //0.9
                    if (Img.Center[Ysite] > COL / 2 + 5) {
                        Img.LeftBorderFindFlag[Ysite] = 'F', Img.RightBorderFindFlag[Ysite] =
                                                                     'F';
                    }
                }
            } else {
                if (circle_flag.out_left_circle == 0) { //环内
                    for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                        Img.Center[Ysite] = Img.RightBorder[Ysite]
                                            - RoadWide0 * 0.5; //0.55
                    }
                } else {
                    if (circle_flag.leave_left_circle == 0) {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW;
                             Ysite++) {
                            Img.Center[Ysite] = (Img.LeftBorder[Ysite]
                                                 + RoadWide0 * 0.5);
                        }
                    } else {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW;
                             Ysite++) {
                            Img.Center[Ysite] = Img.RightBorder[Ysite]
                                                - RoadWide0 * 0.5;
                        }
                    }
                }
            }
        }
        return;
    } else if (circle_flag.find_right_circle == 1) {
        if (circle_flag.enter_right_circle == 0) {
            for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                //此处应对有丢边界的中线进行处理
                if (Img.RightBorderFindFlag[Ysite] == 'T'
                    && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'F'
                         && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线

                    Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 / 2;
                else
                    Img.Center[Ysite] = CenterCol;
            }
        } else {
            if (circle_flag.on_right_circle == 0) {
                for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                    Img.Center[Ysite] = Img.RightBorder[Ysite]
                                        - RoadWide0 * 0.7;
                    if (Img.Center[Ysite] < COL / 2 - 5) {
                        Img.LeftBorderFindFlag[Ysite] = 'F', Img.RightBorderFindFlag[Ysite] =
                                                                     'F';
                    }
                }
            } else {
                if (circle_flag.out_right_circle == 0) {
                    for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                        Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                            + RoadWide0 * 0.45;
                    }
                } else {
                    if (circle_flag.leave_right_circle == 0) {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW;
                             Ysite++) {
                            Img.Center[Ysite] = Img.RightBorder[Ysite]
                                                - RoadWide0 * 0.5;
                        }
                    } else {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW;
                             Ysite++) {
                            Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                                + RoadWide0 * 0.5;
                        }
                    }
                }
            }
        }
        return;
    }

    /*****************三叉************************/

    if (fork_flag.find_fork == 1) {
        if (fork_flag.on_fork == 0) {
            if (fork_flag.leave_fork == 0) {
                if (fork_flag.cnt % 2 == 0) {
                    for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++)
                        Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                            + RoadWide0 * 0.4;
                } else {
                    for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++)
                        Img.Center[Ysite] = Img.RightBorder[Ysite]
                                            - RoadWide0 * 0.4;
                }
            }
        } else {
            if (fork_flag.out_fork == 0) {
                for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
                    //此处应对有丢边界的中线进行处理
                    if (Img.RightBorderFindFlag[Ysite] == 'T'
                        && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    {
                        if (fork_flag.cnt % 2 == 0) {
                            Img.Center[Ysite] = (Img.LeftBorder[Ysite] * 0.6
                                                 + Img.RightBorder[Ysite] * 0.4);
                        } else {
                            Img.Center[Ysite] = (Img.LeftBorder[Ysite] * 0.4
                                                 + Img.RightBorder[Ysite] * 0.6);
                        }
                    } else if (Img.RightBorderFindFlag[Ysite] == 'F'
                               && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
                        Img.Center[Ysite] = CenterCol;
                    else if (Img.RightBorderFindFlag[Ysite] == 'F'
                             && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
                    {
                        Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                            + RoadWide0 * 0.5;
                    } else if (Img.RightBorderFindFlag[Ysite] == 'T'
                               && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
                    {
                        Img.Center[Ysite] = Img.RightBorder[Ysite]
                                            - RoadWide0 * 0.5;
                    }
                }
            } else {
                if (fork_flag.leave_fork == 0) {
                    if (fork_flag.cnt % 2 == 0) {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++)
                            Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                                + RoadWide0 * 0.4;
                    } else {
                        for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++)
                            Img.Center[Ysite] = Img.RightBorder[Ysite]
                                                - RoadWide0 * 0.4;
                    }
                }
            }
        }
    }
    if (door_flag.find_door == 1) {
        for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
            Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 / 2;
        }
        return;
    }
}
int start_COL = 30, start_ROW = 0, end_COL = COL - 30, end_ROW = ROW - 1;
/**************连通域*****************/
Connected_Component connect;
void init_connect() {
    memset(connect.edge, 0, sizeof(connect.edge));
    connect.room = 0;
    connect.edge_len = 3;
}
bool add_edge(int num, uint8_t img_tmp[ROW][COL]) {
    for (int j = 0; j < 2 * connect.edge_len; j++) {
        int num_tmp = num + j;
        int i = (num_tmp + (end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4) %
                ((end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4);
        if (i < end_COL - start_COL + 1) {
            draw(ROW - 1 - start_ROW, start_COL + i, 7);
            my_delay(1);
            if (img_tmp[start_ROW][start_COL + i] == 1)
                return 1;
        } else if (i < end_COL - start_COL + 1 + end_ROW - start_ROW + 1 - 1) {
            draw(ROW - 1 - start_ROW - (i - (end_COL - start_COL)), end_COL, 7);
            my_delay(1);
            if (img_tmp[start_ROW + (i - (end_COL - start_COL))][end_COL] == 1)
                return 1;
        } else if (i < end_COL - start_COL + 1 + end_COL - start_COL + 1 + end_ROW - start_ROW + 1 - 2) {
            draw(ROW - 1 - end_ROW, (end_COL - i + (end_COL - start_COL + 1 + end_ROW - start_ROW + 1 - 2)) - 1, 7);
            my_delay(1);
            if (img_tmp[end_ROW][(end_COL - i + (end_COL - start_COL + 1 + end_ROW - start_ROW + 1 - 2)) - 1] == 1)
                return 1;
        } else {
            draw(ROW - 1 - start_ROW -
                 (end_COL - start_COL + 1 + end_COL - start_COL + 1 + end_ROW - start_ROW + 1 + end_ROW - start_ROW +
                  1 - 4 - i), start_COL + 0, 7);
            my_delay(1);
            if (img_tmp[start_ROW +
                        (end_COL - start_COL + 1 + end_COL - start_COL + 1 + end_ROW - start_ROW + 1 + end_ROW -
                         start_ROW + 1 - 4 - i)][start_COL + 0] == 1)
                return 1;
        }
    }
    return 0;
}
void edge_cnt(uint8_t img_tmp[ROW][COL]) {
    int x = start_COL, y = ROW - 1 - start_ROW;
    int edge_cnt = 0;
    int way = 0; //0 r 1 up 2 l 3 down
    for (int i = 0; i < (end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4; i++) {
        connect.edge[edge_cnt] = ImageUsed[y][x];
        edge_cnt++;
        draw(y, x, 1);
//        my_delay(1);

        if (way == 0)
            x++;
        else if (way == 1)
            y--;
        else if (way == 2)
            x--;
        else if (way == 3)
            y++;
        if (x > end_COL)
            way++, x = end_COL, y = ROW - 1 - start_ROW - 1;
        if (y < ROW - end_ROW - 1)
            way++, x = end_COL - 1, y = ROW - 1 - end_ROW;
        if (x < start_COL)
            way++, x = start_COL, y = ROW - 1 - end_ROW + 1;
        if (y > ROW - 1 - start_ROW)
            way++;
    }
    for (int i = 0; i < (end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4; i++) {
        int sum_start = 0, sum_end = 0;
        for (int j = i; j < connect.edge_len * 2 + i; j++) {
            if (j < (end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4) {
                if (j < i + connect.edge_len)
                    sum_start += connect.edge[j];
                else
                    sum_end += connect.edge[j];
            } else {
                if (j < i + connect.edge_len)
                    sum_start += connect.edge[j - ((end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4)];
                else
                    sum_end += connect.edge[j - ((end_COL - start_COL + 1 + end_ROW - start_ROW + 1) * 2 - 4)];
            }
        }
        if (abs(sum_end - sum_start) == 255 * connect.edge_len
            && add_edge(i, img_tmp))
            connect.room++;
    }
    room_cnt();
}
void room_cnt() {
    if (connect.room == connect.last_room)
        connect.room_cnt++;
    else {
        connect.error_cnt++;
        if (connect.error_cnt >= connect.error_max) {
            connect.room_cnt = 1;
            connect.error_cnt = 0;
            connect.last_room = connect.room;
        }
    }
}
//**********************上交*********************/

const int dir_front[4][2] = {{0,  1}, //上
                             {1,  0},  //右
                             {0,  -1},  //下
                             {-1, 0}};  //左
const int dir_frontright[4][2] = {{1,  1},
                                  {1,  -1},
                                  {-1, -1},
                                  {-1, 1}};
const int dir_frontleft[4][2] = {{-1, 1},//左上
                                 {1,  1},//右上
                                 {1,  -1},//右下
                                 {-1, -1}};//左下
image_t deal_img;
void RUBO_IMG_INIT() {
    deal_img.data = *SimBinImage;
    deal_img.width = COL;
    deal_img.height = ROW;
    deal_img.step = ROW;
}

void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
//    zf_assert(img && img->data);
//    zf_assert(num && *num >= 0);
//    zf_assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < *
            num && half < x && x
                               < img->width - half - 1 && half < y && y < img->height - half - 1 && turn < 4) {
        int local_thres = 0;
        for (
                int dy = -half;
                dy <=
                half;
                dy++) {
            for (
                    int dx = -half;
                    dx <=
                    half;
                    dx++) {
                local_thres +=
                                AT(deal_img, x + dx, y + dy);
            }
        }
        local_thres /=
                block_size * block_size;
        local_thres -=
                clip_value;
        int current_value = AT(deal_img, x, y);
        int front_value = AT(deal_img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = AT(deal_img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {//前不被当，左上被挡直走
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] =
                    x;
            pts[step][1] =
                    y;
            step++;
            ips114_draw_point(x, ROW-1-y, RED);
            my_delay(10);
            turn = 0;
        } else {//左上未被挡，前不被档   直走转左
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] =
                    x;
            pts[step][1] =
                    y;
            step++;
            ips114_draw_point(x, ROW-1-y, RED);
            my_delay(10);
            turn = 0;
        }
    }
    *num = step;
}
void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
//    zf_assert(img && img->data);
//    zf_assert(num && *num >= 0);
//    zf_assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < *num && 0 < x && x < img->width - 1 && 0 < y && y < img->height - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT(deal_img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;
        int current_value = AT(deal_img, x, y);
        int front_value = AT(deal_img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT(deal_img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
            ips114_draw_point(x, ROW-1-y, BLUE);
  my_delay(10);

        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
            ips114_draw_point(x, ROW-1-y, BLUE);
  my_delay(10);
        }
    }
    *num = step;
}
int ipts0[ROW][2], ipts0_num = ROW, ipts1[ROW][2], ipts1_num = ROW;


void process_image() {
    int block_size = 7, clip_value = 2;
    int thres = 150;
    int begin_x = 1;
    int begin_y = 7;
    int x1 = deal_img.width / 2 - begin_x, y1 = begin_y;
    int x2 = deal_img.width / 2 + begin_x, y2 = begin_y;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
//    for (int i = ROW - 1; i >= 0; i--) {
////        if ( ImageUsed[i][Img.LeftBorder[ROW - 1 - i]] != 0)
////            draw(i,  Img.LeftBorder[  ROW - 1 -  i],6);
////        if ( ImageUsed[i][Img.RightBorder[ROW - 1 - i]] != 0)
////            draw(i,Img.RightBorder[ ROW -1 -i],    4);
////       draw(i, Img.Center[ROW - 1 - i], 5);
////        waitKey(1);
////        imshow("result", src);
////    }
    for (; x1 > 0; x1--) if (AT_IMAGE(deal_img, x1 - 1, y1) < thres) break; //x1下一点黑
    if (AT_IMAGE(deal_img, x1, y1) >= thres&&x1!=0)    //左边线，左黑右白
        findline_lefthand_adaptive(&deal_img, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
    else ipts0_num = 0;
    for (; x2 < deal_img.width - 1; x2++) if (AT_IMAGE(deal_img, x2 + 1, y2) < thres) break;
    if (AT_IMAGE(deal_img, x2, y2) >= thres&&x2!=COL-1)
        findline_righthand_adaptive(&deal_img, block_size, clip_value, x2, y2, ipts1, &ipts1_num);
    else ipts1_num = 0;
   int rpts1_num = ipts1_num;

    // 边线滤波
//    blur_points(rpts0, rpts0_num, rpts0b, (int) round(line_blur_kernel));
//   int rpts0b_num = rpts0_num;
//    blur_points(rpts1, rpts1_num, rpts1b, (int) round(line_blur_kernel));
// int   rpts1b_num = rpts1_num;

//    // 边线等距采样
//    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
//    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
//    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
//    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);
//
//    // 边线局部角度变化率
//    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int) round(angle_dist / sample_dist));
//    rpts0a_num = rpts0s_num;
//    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int) round(angle_dist / sample_dist));
//    rpts1a_num = rpts1s_num;
//
//    // 角度变化率非极大抑制
//    nms_angle(rpts0a, rpts0a_num, rpts0an, (int) round(angle_dist / sample_dist) * 2 + 1);
//    rpts0an_num = rpts0a_num;
//    nms_angle(rpts1a, rpts1a_num, rpts1an, (int) round(angle_dist / sample_dist) * 2 + 1);
//    rpts1an_num = rpts1a_num;
//
//    // 左右中线跟踪
//    track_leftline(rpts0s, rpts0s_num, rptsc0, (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
//    rptsc0_num = rpts0s_num;
//    track_rightline(rpts1s, rpts1s_num, rptsc1, (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
//    rptsc1_num = rpts1s_num;
}