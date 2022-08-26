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
    unsigned short i = 0, j = 0;
    uint8_t Threshold1 = ThresholdAdd + OSTUThreshold(CamImage);
    for (i = 0; i < USED_ROW; i++) {
        for (j = 0; j < USED_COL; j++) {
            if (CamImage[i][j] > Threshold1) //数值越大，显示的内容越多，较浅的图像也能显示出来
            {
                SimBinImage[i][j] = 255;
            } else {
                SimBinImage[i][j] = 0;
            }
        }
    }
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
size_point stack_seed2[stack_size1];    //栈
size_point stack_seed3[stack_size1];    //栈
int16 stack_top = 0, stack_top1 = 0, stack_top2 = 0, stack_top3 = 0;
int16 start_line = 10;
int16 left_lost_cnt = 0, right_lost_cnt = 0;
bool stop_flag = 0;
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
        //  {-1, 1},    //左下
        {0,  1},    //下
        {1,  1},    //右下
        {1,  0},    //右
        //   {1,  -1},    //右上
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
bool pull_stack2(int16 x, int16 y) //入栈
{
    if (stack_top2 == stack_size1) {
        search_line_flag = 1;
        return 1;
    }
    stack_seed2[stack_top2].x0 = x;
    stack_seed2[stack_top2].y0 = y;
    stack_top2++;
    return 0;
}
size_point push_stack2() //出栈
{
    stack_seed2[stack_top2].y0 = 0;
    stack_seed2[stack_top2].x0 = 0;
    return stack_seed2[--stack_top2];
}
bool pull_stack3(int16 x, int16 y) //入栈
{
    if (stack_top3 == stack_size1) {
        search_line_flag = 1;
        return 1;
    }
    stack_seed3[stack_top3].x0 = x;
    stack_seed3[stack_top3].y0 = y;
    stack_top3++;
    return 0;
}
size_point push_stack3() //出栈
{
    stack_seed3[stack_top3].y0 = 0;
    stack_seed3[stack_top3].x0 = 0;
    return stack_seed3[--stack_top3];
}
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
int search_start = 0;
const int16 search_start_line = 3;
int go[4][8][2] = {
        //     左,左前,前,右前,右,右后,后,左后                //方位/朝向
        {{-1, 0},  {-1, -1}, {0,  -1}, {1,  -1}, {1,  0},  {1,  1},  {0,  1},
                                                                              {-1, 1}}, //左
        {{0,  -1}, {1,  -1}, {1,  0},  {1,  1},  {0,  1},  {-1, 1},  {-1, 0},
                                                                              {-1, -1}}, //上
        {{1,  0},  {1,  1},  {0,  1},  {-1, 1},  {-1, 0},  {-1, -1}, {0,  -1},
                                                                              {1,  -1}}, //右
        {{0,  1},  {-1, 1},  {-1, 0},  {-1, -1}, {0,  -1}, {1,  -1},
                                                                     {1,  0}, {1,  1}}, //下
};
bool judge_black(int y, int x) {
    if (ImageUsed[ROW - 1 - y][x] == 0)
        return 0;
//
//int sum=0;
//int cnt=0;
//for(int i=y-1;i<=y+1;i++)
//{
//    if(i<ROW&&i>=0)
//
//        for(int j=x-1;j<=x+1;j++)
//    {
//            if(j>=0&&j<COL)
//            {
//                sum+=ImageUsed[ROW-1-i][j];
//                cnt++;
//            }
//    }
//}
//    if( sum/cnt<100)return 0;



    return 1;
}
bool blocked(int head, int dir, int center_x, int center_y) {
    center_x = go[head][dir][1] + center_x;
    center_y = go[head][dir][0] + center_y;
    if (center_x < 0 || center_x >= COL)
        return 1;
    if (center_y < 0 || center_y >= ROW)
        return 1;
    if (!judge_black(center_y, center_x))return 1;
    return 0;
}
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}
// 点集三角滤波
void blur_points(int16 pts_in[][2], int num, float pts_out[][2], int kernel) {
    int half = kernel / 2; //3
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) { //-3~~3 (-2~4)-(0~3)
            pts_out[i][0] += 1.0 * pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += 1.0 * pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}
bool check_line(uint8_t img_tmp[ROW][COL], int x, int y) {
    if (x == COL - 1 && y == 0) {
        int a = 0;
    }
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            if (y + i < ROW && y + i >= 0 && x + j < COL && x + j >= 0)
                if (img_tmp[y + i][x + j])
                    return 1;
    return 0;
}
//clip

void search_line(void) {
    for (int i = 0; i < ROW; i++) {
        Img.LeftBorder[i] = 0, Img.RightBorder[i] = COL - 1;
    }
    int16 px, py;
    search_line_flag = 0;
    uint8_t img_tmp[ROW][COL];
    size_point center_seed;
    stack_top = 0, stack_top1 = 0;
    start_line = search_start_line;
    int search_len = 20;
    int turn_flag = 0;
    seed_left_x = 0, seed_left_y = 0;
    seed_right_x = 0, seed_right_y = 0;
    init_connect();
    bool down_flag = 0;
    int search_start_sum = 0, search_start_sum_cnt = 0;
    int find = 0;
    for (int i = 0; i < COL; i++) {
        if (ImageUsed[ROW - 1 - find][i] != 0) {
            search_start_sum += i;
            search_start_sum_cnt++;
        }
    }
//    if(!circle_left)
//    search_start = search_start_sum / search_start_sum_cnt+30>=COL?COL-1: search_start_sum / search_start_sum_cnt+30 ;
//    else     search_start = search_start_sum / search_start_sum_cnt-30>=0? search_start_sum / search_start_sum_cnt-30 :0;

    memset(img_tmp, 0, sizeof(img_tmp));
    while (start_line >= 0 //行仲未负
           && (stack_top == 0 || stack_top1 == 0 //左右点仲未揾到
               || (seed_right_x - seed_left_x <= 10)) //或者两点太贴近，继续揾种子
           && (start_line < ROW - 1)) {//行唔得超过ROW
        stack_top = 0, stack_top1 = 0;
        seed_right_x = 0, seed_left_x = 0;
        for (int i =
                (search_start + search_len) < COL - 3 ?
                (search_start + search_len) : COL - 3; i >= 0; i--) { //揾左点
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i + 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0) //条件1:黑(i)+白(i+1)
                || (i == 0 && ImageUsed[ROW - 1 - start_line][i] != 0)//条件2:左边界，且系白
                    ) {
                if (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0) //条件2得行
                {
                    if (pull_stack(i, start_line, img_tmp))
                        return;
                    seed_left_x = i;
                    seed_left_y = start_line;
                    break;
                }
                if (ImageUsed[ROW - 1 - start_line][i + 1]
                    == ImageUsed[ROW - 1 - start_line][i + 2] //条件3:一黑+二白(i+1,i+2)系左边界。
                        ) {
                    if (pull_stack(i + 1, start_line, img_tmp))
                        return;
                    seed_left_x = i + 1;
                    seed_left_y = start_line;
                    break;
                }
            }
        }
        for (int i =
                search_start - search_len >= 2 ? search_start - search_len : 2;
             i <= COL - 1; i++) {    //揾右点
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i - 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0)   //条件1:白(i-1)+黑(i)
                || (i == COL - 1 && ImageUsed[ROW - 1 - start_line][i] != 0) //条件2:右边界，且系白
                    ) {
                if (ImageUsed[ROW - 1 - start_line][i] != 0
                    && i == COL - 1) {       //条件2得行
                    if (pull_stack1(i, start_line, img_tmp))
                        return;
                    seed_right_x = i;
                    seed_right_y = start_line;
                    break;
                } else if (ImageUsed[ROW - 1 - start_line][i - 1]
                           == ImageUsed[ROW - 1 - start_line][i - 2] //条件3:二白(i-1,i-2)+一黑系右边界。
                        ) {
                    if (pull_stack1(i - 1, start_line, img_tmp))
                        return;
                    seed_right_x = i - 1;
                    seed_right_y = start_line;
                    break;
                }
            }
        }
        if (seed_right_x - seed_left_x <= 10 && turn_flag == 0) { //，佢哋唔应靠太近，可能系车房 ，揾到异常啲，开始向上头揾点
            turn_flag = 1;
            door_flag.find_line = 1;
        }
        if (turn_flag == 0) start_line--; //仲未揾到异常啲，向下揾种子
        else if (turn_flag == 1) {  //已经揾到异常啲，向上揾种子
            if (seed_left_x != 0 && seed_right_x != 0) { //都揾到，但唔得，从佢哋上一行向上揾
                start_line = start_line + 1, turn_flag++;
            } else { //下面都揾唔到，死咗~
                start_line = -120;
                break;
            }
        } else start_line++; //继续向上揾
    }
    if (stack_top == 0 || stack_top1 == 0 || start_line < 0 //摄像头保护
        || search_line_flag == 1 || start_line > ROW - 1) {
//        ExSpeed = 0;
//        if (stop_flag == 0) {
//            ips114_clear(GREEN);
//        }
//        stop_flag = 1;
//        gpio_low(BEEP_PIN);
//        ips114_show_string(1, 1, "speed die!");
        return;
    }
    max_py = 0, max_py1 = 0;
    min_py = 99, min_py1 = 99;
    up_left_flag = 0, up_right_flag = 0;
    up_left_num = 0, up_right_num = 0;
    if (line_debug) {
        ips114_clear(YELLOW);
    }
    while ((stack_top + stack_top1 != 0 && ((up_left_num != up_right_num) || !(max_py == max_py1 && max_py == 99))) ||
           (down_flag && stack_top + stack_top1 != 0 && !(min_py == min_py1 && min_py == 0))) {
        while (stack_top != 0 && (((max_py <= max_py1 && !up_left_flag) || stack_top1 == 0) ||
                                  (up_left_flag && up_left_num <= up_right_num) ||
                                  (down_flag && stack_top != 0 && (stack_top1 == 0 || min_py >= min_py1)))) {
            center_seed = push_stack();
            for (int i = 0; i < 8; i++) {
                px = center_seed.x0 + connects[i].x0;
                py = center_seed.y0 + connects[i].y0;
                if (!(px < 0 || py < 0 || px >= COL || py >= ROW)) {
                    if (img_tmp[py][px] == 0) {
                        if (ImageUsed[ROW - 1 - py][px] != ImageUsed[ROW - 1 - center_seed.y0][center_seed.x0] ||
                            (((px == 0 || (py == 0 && i != 6)) && ImageUsed[ROW - 1 - py][px] != 0) ||
                             (py == ROW - 1 && ImageUsed[ROW - 1 - py][px] != 0))) {
                            if (pull_stack(px, py, img_tmp)) return;
                            if (line_debug) {
                                ips114_draw_point(px, ROW - 1 - py, RED);
                                my_delay(10);
                            }
                            if (py == ROW - 1 && ImageUsed[ROW - 1 - py][px] != 0) up_left_flag = 1;
                            if (up_left_flag) up_left_num++;
                            while (px > 0 && ImageUsed[ROW - 1 - py][px - 1] != 0) px--;
                            while (px < COL - 1 && ImageUsed[ROW - 1 - py][px] == 0) px++;
                            if (Img.LeftBorder[py] < px) Img.LeftBorder[py] = px;
                            if (py > max_py) max_py = py;
                            if (py < min_py1) min_py1 = py;
                        }
                    }
                }
            }
        }
        while (stack_top1 != 0 && (((max_py >= max_py1 && !up_right_flag) || stack_top == 0) ||
                                   (up_right_flag && up_right_num <= up_left_num))) {
            center_seed = push_stack1();
            for (int i = 0; i < 8; i++) {
                px = center_seed.x0 + connects[i].x0;
                py = center_seed.y0 + connects[i].y0;
                if (!(px < 0 || py < 0 || px >= COL || py >= ROW)) {
                    if (img_tmp[py][px] == 0) {
                        if (ImageUsed[ROW - 1 - py][px] != ImageUsed[ROW - 1 - center_seed.y0][center_seed.x0] ||
                            (((px == COL - 1 || (py == 0 && i != 2)) && ImageUsed[ROW - 1 - py][px] != 0) ||
                             (py == ROW - 1 && ImageUsed[ROW - 1 - py][px] != 0))) {
                            if (pull_stack1(px, py, img_tmp)) return;
                            if (line_debug) {
                                ips114_draw_point(px, ROW - 1 - py, BLUE);
                                my_delay(10);
                            }
                            if (py == ROW - 1 && ImageUsed[ROW - 1 - py][px] != 0) up_right_flag = 1;
                            if (up_right_flag)up_right_num++;
                            while (px < COL - 1 && ImageUsed[ROW - 1 - py][px + 1] != 0) px++;
                            while (px > 0 && ImageUsed[ROW - 1 - py][px] == 0) px--;
                            if (Img.RightBorder[py] > px) Img.RightBorder[py] = px;
                            if (py > max_py1) max_py1 = py;
                            if (py < min_py1) min_py1 = py;
                        }
                    }
                }
            }
        }
        if (!(stack_top + stack_top1 != 0 &&
              ((up_left_num != up_right_num) || !(max_py == max_py1 && max_py == 99))))
            down_flag = 1;
    }
    left_lost_cnt = 0, right_lost_cnt = 0;
    for (int i = 0; i < ROW; i++) {
        if (Img.LeftBorder[i] <= 0 + lose_error
            || Img.LeftBorder[i] >= COL - 1 - lose_error)
            Img.LeftBorderFindFlag[i] = 'F', left_lost_cnt++;
        else Img.LeftBorderFindFlag[i] = 'T';
        if (Img.RightBorder[i] <= 0 + lose_error
            || Img.RightBorder[i] >= COL - 1 - lose_error)
            Img.RightBorderFindFlag[i] = 'F', right_lost_cnt++;
        else Img.RightBorderFindFlag[i] = 'T';
    }
    int k_num = 3;
    float k_excel[COL];
    for (int i = 0; i < COL; i++) k_excel[i] = 1.0 * i / 10;
    for (int i = 0; i < ROW; i++) {
        if (i < 0 + k_num || i >= ROW - k_num) Img.RoadWide[i] = RoadWide0;
        else {
            if (Img.RightBorderFindFlag[i] == 'T' && Img.LeftBorderFindFlag[i] == 'T') Img.RoadWide[i] = RoadWide0;
            else if (Img.RightBorderFindFlag[i] == 'T' && Img.RightBorderFindFlag[i + k_num] == 'T' &&
                     Img.RightBorderFindFlag[i - k_num] == 'T' && Img.LeftBorderFindFlag[i] == 'F')
                Img.RoadWide[i] =
                        RoadWide0 * (1 + k_excel[abs(Img.RightBorder[i + k_num] - Img.RightBorder[i - k_num])]);
            else if (Img.RightBorderFindFlag[i] == 'F' && Img.LeftBorderFindFlag[i] == 'T' &&
                     Img.LeftBorderFindFlag[i - k_num] == 'T' && Img.LeftBorderFindFlag[i + k_num] == 'T')
                Img.RoadWide[i] = RoadWide0 * (1 + k_excel[abs(Img.LeftBorder[i + k_num] - Img.LeftBorder[i - k_num])]);
            else Img.RoadWide[i] = RoadWide0;
        }
    }
//    edge_cnt(img_tmp);
    //  GetBorder( img_tmp);
    //cout <<
    if (circle_left) find_single_inflection('R', img_tmp);// << endl;
    else find_single_inflection('L', img_tmp);// << endl;
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
int error_find = 0;
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
int view_tmp = 0;
void solve_line() {
    float kk = 1.2;
// int CenterCalMaxRow = CenterCalMaxRow_tmp; //15
    error_find = 0;
    if (circle_flag.find_left_circle == 0 && circle_flag.find_right_circle == 0
        && fork_flag.find_fork == 0 && Cross.FindFlag == 'F'
        && door_flag.find_door == 0) {
        for (int Ysite = StartScanRow; Ysite < ROW; Ysite++) {
            if (Img.RightBorder[Ysite] < Img.LeftBorder[Ysite]) {
                Img.LeftBorderFindFlag[Ysite] = 'F';
                Img.RightBorderFindFlag[Ysite] = 'F';
                if (error_find == 0) {
                    CenterCalMaxRow = Ysite;
                    CenterCalMaxRow = Ysite;
                    error_find = 1;
                }
            }
            if (((Img.RightBorderFindFlag[Ysite] == 'T'
                  && Img.LeftBorderFindFlag[Ysite] == 'F')
                 || (Img.RightBorderFindFlag[Ysite] == 'F'
                     && Img.LeftBorderFindFlag[Ysite] == 'T'))
                && (Img.RightBorder[Ysite] - Img.LeftBorder[Ysite]
                    >= RoadWide0 * 2)
                && (Img.LeftBorder[Ysite] < lose_error
                    && Img.RightBorder[Ysite] > COL - 1 - lose_error)) {
                if (error_find == 0) {
                    CenterCalMaxRow = Ysite;
                    CenterCalMaxRow = Ysite;
                    error_find = 1;
                }
                if (Img.LeftBorderFindFlag[Ysite] == 'T')
                    left_lost_cnt++;
                if (Img.RightBorderFindFlag[Ysite] == 'T')
                    right_lost_cnt++;
                Img.LeftBorderFindFlag[Ysite] = 'F';
                Img.RightBorderFindFlag[Ysite] = 'F';
            }
            if (Ysite < StartScanRow + 1) {
                if (Img.RightBorderFindFlag[Ysite] == 'T'
                    && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    Img.Center[Ysite] = (Img.LeftBorder[Ysite]
                                         + Img.RightBorder[Ysite]) / 2;
                else if (Img.RightBorderFindFlag[Ysite] == 'F'
                         && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
                    Img.Center[Ysite] = CenterCol;
                else if (Img.RightBorderFindFlag[Ysite] == 'F'
                         && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
                    Img.Center[Ysite] = Img.LeftBorder[Ysite]
                                        + Img.RoadWide[Ysite] / 2
                                        + (search_start - COL / 2)
                                          * (0
                                             + kk
                                               * (1.0
                                                  - 1.0
                                                    * max(max_py,
                                                          max_py1)
                                                    / ROW));
                else if (Img.RightBorderFindFlag[Ysite] == 'T'
                         && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
                    Img.Center[Ysite] = Img.RightBorder[Ysite]
                                        - Img.RoadWide[Ysite] / 2
                                        + (search_start - COL / 2)
                                          * (0
                                             + kk
                                               * (1.0
                                                  - 1.0
                                                    * max(max_py,
                                                          max_py1)
                                                    / ROW));
            } else {
                if (Img.RightBorderFindFlag[Ysite] == 'T'
                    && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
                    Img.Center[Ysite] = (Img.LeftBorder[Ysite]
                                         + Img.RightBorder[Ysite]) / 2 * 0.9
                                        + 0.1 * Img.Center[Ysite - 1];
                else if (Img.RightBorderFindFlag[Ysite] == 'F'
                         && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
                    Img.Center[Ysite] = CenterCol;
                else if (Img.RightBorderFindFlag[Ysite] == 'F'
                         && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
                    Img.Center[Ysite] = (Img.LeftBorder[Ysite]
                                         + Img.RoadWide[Ysite] / 2) * 0.9
                                        + 0.1 * Img.Center[Ysite - 1]
                                        + (search_start - COL / 2)
                                          * (0
                                             + kk
                                               * (1.0
                                                  - 1.0
                                                    * max(max_py,
                                                          max_py1)
                                                    / ROW));
                else if (Img.RightBorderFindFlag[Ysite] == 'T'
                         && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
                    Img.Center[Ysite] = (Img.RightBorder[Ysite]
                                         - Img.RoadWide[Ysite] / 2) * 0.9
                                        + 0.1 * Img.Center[Ysite - 1]
                                        + (search_start - COL / 2)
                                          * (0
                                             + kk
                                               * (1.0
                                                  - 1.0
                                                    * max(max_py,
                                                          max_py1)
                                                    / ROW));
            }
        }
        return;
    }
}
single_inflection single_inflection_flag;
int connect_line(int x1, int y1, int x2, int y2, char way) {
    int i = 0;
    float k, b;
    int y_max, y_min;
    y_max = max_ab(y1, y2);
    y_min = min_ab(y1, y2);
    k = 1.0 * (y2 - y1) / (x2 - x1); //直线
    b = 1.0 * (((y2 + y1) - k * (x2 + x1))) / 2;
    for (i = y_min; i <= y_max; i++) {
        if (way == 'L')Img.LeftBorder[i] = (int) (1.0 * (i - b) / k);
        else Img.RightBorder[i] = (int) (1.0 * (i - b) / k);
    }
}
bool inflection_judge(int y, int x, uint8_t img_tmp[ROW][COL]) {
    for (int i = x - 3; i <= x + 3; i++)
        if (i > 0 && i < COL)
            if (img_tmp[y][i])return 1;
    return 0;
}
int check_inflection(int max_row, bool find[ROW], int start_x[ROW], uint8_t black_cnt[ROW], int check_len,
                     int &find_start) {
    int cnt_tmp = 0;
    for (int i = 1; i < 5; i++)
        if (i + max_row < ROW && !find[i + max_row])return 0;
    for (int i = 0; i < 5; i++)
        if (i + max_row < ROW - 1 && black_cnt[i + 1 + max_row] + 1 >= black_cnt[i + max_row]) cnt_tmp++;
    if (max_row < ROW - 1 && abs(start_x[max_row] - start_x[max_row + 1]) > 10) {
        if ((find_start) == 0) (find_start) = max_row + 2;
        return 0;
    }
    if (cnt_tmp <= 4)return 0;
    return 1;
}
bool find_single_inflection(char start_way, uint8_t img_tmp[ROW][COL]) {
    uint8_t black_cnt[ROW] = {0};
    int start_x[ROW] = {0};
    bool find[ROW] = {0};
    single_inflection_flag.find = 0;
    if (start_way == 'L') {
        for (int i = 0; i < ROW; i++) {
            for (int j = 0; j < COL - 3; j++) {
                if (ImageUsed[ROW - 1 - i][j] == ImageUsed[ROW - 1 - i][j + 1] &&
                    ImageUsed[ROW - 1 - i][j + 2] == ImageUsed[ROW - 1 - i][j + 3] &&//2 white & 2 black
                    ImageUsed[ROW - 1 - i][j] != 0 && ImageUsed[ROW - 1 - i][j + 2] != ImageUsed[ROW - 1 - i][j] &&
                    inflection_judge(i, j, img_tmp)) {
                    j += 2; //ImageUsed[ROW - 1 - i][j] is black
                    int start_tmp = j;
                    while (j < COL - 2 && !(ImageUsed[ROW - 1 - i][j] == 0 && ImageUsed[ROW - 1 - i][j + 1] != 0 &&
                                            ImageUsed[ROW - 1 - i][j + 2] != 0 && inflection_judge(i, j, img_tmp))) {
                        black_cnt[i]++;
                        j++;
                    }
                    if (j != COL - 3) //find point
                    {
                        find[i] = 1;
                        start_x[i] = start_tmp;
                        break;
                    } else {
                        find[i] = 0;
                        break;
                    }
                }
            }
        }
        int find_start = 0;
        bool check_point[ROW] = {0};
        for (int i = find_start; i < ROW; i++) {
            if (find[i])
                draw(ROW - 1 - i, start_x[i], 1);
            if (check_inflection(i, find, start_x, black_cnt, 8, find_start) && find[i]) {
                check_point[i] = 1;
            }
        }
        for (int i = find_start; i < ROW; i++) {
            if (check_point[i] == 1) {
                single_inflection_flag.find = 1;
                single_inflection_flag.now_prev_point_x = start_x[i];
                single_inflection_flag.now_prev_point_y = i;
                break;
            }
        }
    } else if (start_way == 'R')  //右边开始搜索
    {
        for (int i = 0; i < ROW; i++) {
            for (int j = COL - 1; j >= 3; j--) {
                if (ImageUsed[ROW - 1 - i][j] == ImageUsed[ROW - 1 - i][j - 1] &&
                    ImageUsed[ROW - 1 - i][j - 2] == ImageUsed[ROW - 1 - i][j - 3] &&//2 white & 2 black
                    ImageUsed[ROW - 1 - i][j] != 0 && ImageUsed[ROW - 1 - i][j - 2] != ImageUsed[ROW - 1 - i][j] &&
                    inflection_judge(i, j, img_tmp)) {
                    j -= 2; //ImageUsed[ROW - 1 - i][j] is black
                    int start_tmp = j;
                    while (j >= 2 && !(ImageUsed[ROW - 1 - i][j] == 0 && ImageUsed[ROW - 1 - i][j - 1] != 0 &&
                                       ImageUsed[ROW - 1 - i][j - 2] != 0 && inflection_judge(i, j, img_tmp))) {
                        black_cnt[i]++;
                        j--;
                    }
                    if (j != 2) //find point
                    {
                        find[i] = 1;
                        start_x[i] = start_tmp;
                        break;
                    } else {
                        find[i] = 0;
                        break;
                    }
                }
            }
        }
        int find_start = 0;
        bool check_point[ROW] = {0};
        for (int i = find_start; i < ROW; i++) {
//        if(find[i])
//            draw(ROW - 1 - i, start_x[i], 1);

            if (check_inflection(i, find, start_x, black_cnt, 8, find_start) && find[i]) {
                check_point[i] = 1;
            }
        }
        for (int i = find_start; i < ROW; i++) {
            if (check_point[i] == 1) {
                single_inflection_flag.find = 1;
                single_inflection_flag.now_prev_point_x = start_x[i];
                single_inflection_flag.now_prev_point_y = i;
                break;
            }
        }
    }
    if (start_way == 'R')
        connect_line(Img.RightBorder[0], 0, single_inflection_flag.now_prev_point_x,
                     single_inflection_flag.now_prev_point_y, 'R');
    else if (start_way == 'L')
        connect_line(Img.LeftBorder[0], 0, single_inflection_flag.now_prev_point_x,
                     single_inflection_flag.now_prev_point_y, 'L');
    draw(ROW - 1 - single_inflection_flag.now_prev_point_y, single_inflection_flag.now_prev_point_x, 3);
    return 0;
}
void search_line_fork(void) {
    for (int i = 0; i < ROW; i++)
        Img.LeftBorder[i] = 0, Img.RightBorder[i] = COL - 1;
    uint8_t img_tmp[ROW][COL];
    stack_top = 0, stack_top1 = 0;
    start_line = search_start_line;
    int search_len = 20;
    int turn_flag = 0;
    seed_left_x = 0, seed_left_y = 0;
    seed_right_x = 0, seed_right_y = 0;
    init_connect();
    bool down_flag = 0;
    Img.right_line_cnt = 0;
    Img.left_line_cnt = 0;
    Img.left_cnt_itcm = 0;
    Img.right_cnt_itcm = 0;
    Img.left_edge_cnt = 0;
    Img.right_edge_cnt = 0;
    Img.left_cnt_deal = 0;
    Img.right_cnt_deal = 0;
    int search_start_sum = 0, search_start_sum_cnt = 0;
    for (int i = 0; i < COL; i++) {
        if (judge_black(0, i) != 0) {
            search_start_sum += i;
            search_start_sum_cnt++;
        }
    }
    search_start = search_start_sum / search_start_sum_cnt;
    memset(img_tmp, 0, sizeof(img_tmp));
    while (start_line >= 0
           && (stack_top == 0 || stack_top1 == 0
               || (seed_right_x - seed_left_x <= 10))
           && (start_line < ROW - 1)) {
        stack_top = 0, stack_top1 = 0;
        seed_right_x = 0, seed_left_x = 0;
        for (int i =
                (search_start + search_len) < COL - 3 ?
                (search_start + search_len) : COL - 3; i >= 0; i--) {
            if ((judge_black(start_line, i)
                 != judge_black(start_line, i + 1)
                 && judge_black(start_line, i) == 0)
                || (i == 0 && judge_black(start_line, i) != 0)
                || (judge_black(start_line, i) != 0 && i == 0)) {
                if (judge_black(start_line, i + 1)
                    == judge_black(start_line, i + 2)
                        ) {
                    if (judge_black(start_line, i) != 0 && i == 0) {
                        if (pull_stack(i, start_line, img_tmp))
                            return;
                        img_tmp[start_line][i + 1] = 1;
                        seed_left_x = i;
                        seed_left_y = start_line;
                        break;
                    } else if (pull_stack(i + 1, start_line, img_tmp))
                        return;
                    img_tmp[start_line][i + 1] = 1;
                    seed_left_x = i + 1;
                    seed_left_y = start_line;
                    break;
                }
            }
        }
        for (int i =
                search_start - search_len >= 2 ? search_start - search_len : 2;
             i <= COL - 1; i++) {
            if ((judge_black(start_line, i)
                 != judge_black(start_line, i - 1)
                 && judge_black(start_line, i) == 0)
                || (i == COL - 1 && judge_black(start_line, i) != 0)
                || (judge_black(start_line, i) != 0 && i == COL - 1)) {
                if (judge_black(start_line, i) != 0
                    && i == COL - 1) {
                    if (pull_stack1(i, start_line, img_tmp))
                        return;
                    seed_right_x = i;
                    seed_right_y = start_line;
                    break;
                } else if (judge_black(start_line, i - 1)
                           == judge_black(start_line, i - 2)
                        ) {
                    if (pull_stack1(i - 1, start_line, img_tmp))
                        return;
                    seed_right_x = i - 1;
                    seed_right_y = start_line;
                    break;
                }
            }
        }
        if (seed_right_x - seed_left_x <= 10 && turn_flag == 0) {
            turn_flag = 1;
            door_flag.find_line = 1;
        }
        if (turn_flag == 0) start_line--;
        else if (turn_flag == 1) {
            if (seed_left_x != 0 && seed_right_x != 0) {
                start_line = search_start_line + 10, turn_flag++;
            } else {
                start_line = -120;
                break;
            }
        } else start_line++;
    }
//     if (stack_top == 0 || stack_top1 == 0 || start_line < 0 //����ͷ����
//         || search_line_flag == 1 || start_line > ROW - 1) {
//         ExSpeed = 0;
//         if (stop_flag == 0) {
//             ips114_clear(GREEN);
//         }
//         stop_flag = 1;
//         gpio_low(BEEP_PIN);
//         ips114_show_string(1, 1, "speed die!");
//         return;
//     }
    up_left_flag = 0, up_right_flag = 0;
    up_left_num = 0, up_right_num = 0;
    int head_left = 0, head_right = 2;
    int left_center_x = seed_left_x, left_center_y = seed_left_y;
    int right_center_x = seed_right_x, right_center_y = seed_right_y;
    Img.LeftBorder[left_center_y] = left_center_x;
    Img.RightBorder[right_center_y] = right_center_x;
    max_py = left_center_y, max_py1 = right_center_y;
    min_py = left_center_y, min_py1 = right_center_y;
    pull_stack2(left_center_x, left_center_y);
    pull_stack3(right_center_x, right_center_y);
    int min_left_x, min_left_y;
    if (line_debug) {
        ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 3);
        ips114_draw_point(right_center_x, ROW - 1 - right_center_y, 1);
        my_delay(10);
    }
    while ((left_center_x != right_center_x || right_center_y != left_center_y)) {
        //left
        if (((max_py <= max_py1 && !up_left_flag) || (up_left_flag && up_left_num <= up_right_num)) && min_py1 != 0 &&
            down_flag
            ||
            ((max_py <= max_py1 && !up_left_flag) || (up_left_flag && up_left_num <= up_right_num)) && down_flag == 0
            || down_flag == 1 && min_py1 != 0) {
            while (blocked(head_left, 2, left_center_x, left_center_y)) head_left = (head_left + 5) % 4;
            if (!blocked(head_left, 1, left_center_x, left_center_y)) {
                left_center_x += go[head_left][1][1];
                left_center_y += go[head_left][1][0];
                if (left_center_x == seed_left_x && left_center_y == seed_left_y) return;
                if (left_center_x == right_center_x && right_center_y == left_center_y && !down_flag) {
                    down_flag = 1;
                    head_left = 2, head_right = 0;
                    left_center_x = seed_right_x, left_center_y = seed_right_y;
                    right_center_x = seed_left_x, right_center_y = seed_left_y;
                    continue;
                }
                head_left = (head_left + 3) % 4;
                if (up_left_flag == 1 && !down_flag) up_left_num++;
            } else if (blocked(head_left, 1, left_center_x, left_center_y)) {
                left_center_x += go[head_left][2][1];
                left_center_y += go[head_left][2][0];
                if (left_center_x == seed_left_x && left_center_y == seed_left_y) return;
                if (left_center_x == right_center_x && right_center_y == left_center_y && !down_flag) {
                    down_flag = 1;
                    head_left = 2, head_right = 0;
                    left_center_x = seed_right_x, left_center_y = seed_right_y;
                    right_center_x = seed_left_x, right_center_y = seed_left_y;
                    continue;
                }
                if (up_left_flag == 1 && !down_flag) up_left_num++;
            }
            if (max_py < left_center_y) {
                max_py = left_center_y;
                if (max_py == ROW - 1) up_left_flag = 1;
            }
            if (min_py1 > left_center_y && down_flag) {
                min_py1 = left_center_y;
                if (min_py == min_py1 && min_py == 0) {
                    img_tmp[left_center_y][left_center_x] = 1;
                    break;
                }
            }
            img_tmp[left_center_y][left_center_x] = 1;
            if (!down_flag) {
                Img.Left_line[Img.left_line_cnt][0] = left_center_x;
                Img.Left_line[Img.left_line_cnt][1] = left_center_y;
                Img.left_line_cnt++;
                if (!up_left_flag) Img.left_edge_cnt = Img.left_line_cnt;
                if (line_debug) {
                    ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 3);
                    my_delay(10);
                }
            } else {
                pull_stack3(left_center_x, left_center_y);
                if (line_debug) {
                    ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 1);
                    my_delay(10);
                }
            }
        }



        ///right
        if (((max_py1 <= max_py && !up_right_flag) || (up_right_flag && up_left_num >= up_right_num)) && min_py != 0 ||
            ((max_py1 <= max_py && !up_right_flag) || (up_right_flag && up_left_num >= up_right_num)) && !down_flag
            || down_flag == 1 && min_py != 0) {
            while (blocked(head_right, 2, right_center_x, right_center_y))
                head_right = (head_right + 3) % 4;
            if (!blocked(head_right, 3, right_center_x, right_center_y)) {
                right_center_x += go[head_right][3][1];
                right_center_y += go[head_right][3][0];
                if (right_center_x == seed_right_x && right_center_y == seed_right_y) return;
                if (left_center_x == right_center_x && right_center_y == left_center_y && !down_flag) {
                    down_flag = 1;
                    head_left = 2, head_right = 0;
                    left_center_x = seed_right_x, left_center_y = seed_right_y;
                    right_center_x = seed_left_x, right_center_y = seed_left_y;
                    continue;
                }
                head_right = (head_right + 5) % 4;
                if (up_right_flag == 1 && !down_flag) up_right_num++;
            } else if (blocked(head_right, 3, right_center_x, right_center_y)) {
                right_center_x += go[head_right][2][1];
                right_center_y += go[head_right][2][0];
                if (right_center_x == seed_right_x && right_center_y == seed_right_y) return;
                if (left_center_x == right_center_x && right_center_y == left_center_y && !down_flag) {
                    down_flag = 1;
                    head_left = 2, head_right = 0;
                    left_center_x = seed_right_x, left_center_y = seed_right_y;
                    right_center_x = seed_left_x, right_center_y = seed_left_y;
                    continue;
                }
                if (up_right_flag == 1 && !down_flag) up_right_num++;
            }
            if (max_py1 < right_center_y) {
                max_py1 = right_center_y;
                if (max_py1 == ROW - 1) up_right_flag = 1;
            }
            if (min_py > right_center_y && down_flag) {
                min_py = right_center_y;
                if (min_py == min_py1 && min_py1 == 0) {
                    if (Img.RightBorder[left_center_y] > left_center_x)
                        Img.RightBorder[left_center_y] = left_center_x;
                    if (Img.LeftBorder[right_center_y] < right_center_x)
                        Img.LeftBorder[right_center_y] = right_center_x;
                    min_left_x = right_center_x, min_left_y = right_center_y;
                    img_tmp[right_center_y][right_center_x] = 1;
                    break;
                }
            }
            img_tmp[right_center_y][right_center_x] = 1;
            if (!down_flag) {
                Img.Right_line[Img.right_line_cnt][0] = right_center_x;
                Img.Right_line[Img.right_line_cnt][1] = right_center_y;
                Img.right_line_cnt++;
                if (!up_right_flag) Img.right_edge_cnt = Img.right_line_cnt;
                if (line_debug) {
                    ips114_draw_point(right_center_x,
                                      ROW - 1 - right_center_y, 1);
                    my_delay(10);
                }
            } else {
                pull_stack2(right_center_x, right_center_y);
                if (line_debug) {
                    ips114_draw_point(right_center_x, ROW - 1 - right_center_y, 3);
                    my_delay(10);
                }
            }
        }
    }
    //��¼����
    pull_stack2(min_left_x, min_left_y);
    int tmp_len = stack_top2;
    Img.left_cnt_deal = tmp_len + Img.left_line_cnt;
    Img.left_edge_cnt += tmp_len;
    if (max_py == ROW - 1) Img.left_edge_cnt++;
    for (int i = 0; i < Img.left_cnt_deal; i++) {
        if (i < tmp_len) {
            size_point center_seed = push_stack2();
            Img.left_deal_line[i][0] = center_seed.x0;
            Img.left_deal_line[i][1] = center_seed.y0;
        } else {
            Img.left_deal_line[i][0] = Img.Left_line[i - tmp_len][0];
            Img.left_deal_line[i][1] = Img.Left_line[i - tmp_len][1];
        }
    }
    tmp_len = stack_top3;
    Img.right_cnt_deal = tmp_len + Img.right_line_cnt;
    Img.right_edge_cnt += tmp_len;
    if (max_py1 == ROW - 1) Img.right_edge_cnt++;
    for (int j = 0; j < Img.right_cnt_deal; j++) {
        if (j < tmp_len) {
            size_point center_seed = push_stack3();
            Img.right_deal_line[j][0] = center_seed.x0;
            Img.right_deal_line[j][1] = center_seed.y0;
        } else {
            Img.right_deal_line[j][0] = Img.Right_line[j - tmp_len][0];
            Img.right_deal_line[j][1] = Img.Right_line[j - tmp_len][1];
        }
    }
    blur_points(Img.left_deal_line, Img.left_cnt_deal, Img.Left_line_itcm, 7);
    blur_points(Img.right_deal_line, Img.right_cnt_deal, Img.Right_line_itcm, 7);
    Img.left_cnt_itcm = Img.left_line_cnt;
    Img.right_cnt_itcm = Img.right_line_cnt;
    //�������ұ���



    for (int i = 0; i <= Img.left_edge_cnt; i++) {
        if((int) round(Img.Left_line_itcm[i][1])<0||(int) round(Img.Left_line_itcm[i][1])>=ROW)continue;
        if(round(Img.Left_line_itcm[i][0])<0||round(Img.Left_line_itcm[i][0])>=COL)continue;
        if (Img.LeftBorder[(int) round(Img.Left_line_itcm[i][1])] < round(Img.Left_line_itcm[i][0]))
            Img.LeftBorder[(int) round(Img.Left_line_itcm[i][1])] = round(Img.Left_line_itcm[i][0]);
    }
    for (int i = 0; i < Img.right_edge_cnt; i++) {
        if((int) round(Img.Right_line_itcm[i][1])<0||(int) round(Img.Right_line_itcm[i][1])>=ROW)continue;
        if(round(Img.Right_line_itcm[i][0])<0||round(Img.Right_line_itcm[i][0])>=COL)continue;
        if (Img.RightBorder[(int) round(Img.Right_line_itcm[i][1])] > round(Img.Right_line_itcm[i][0]))
            Img.RightBorder[(int) round(Img.Right_line_itcm[i][1])] = round(Img.Right_line_itcm[i][0]);
    }
    left_lost_cnt = 0, right_lost_cnt = 0;
    for (int i = 0; i < ROW; i++) {
        if (Img.LeftBorder[i] <= 0 + lose_error
            || Img.LeftBorder[i] >= COL - 1 - lose_error)
            Img.LeftBorderFindFlag[i] = 'F', left_lost_cnt++;
        else
            Img.LeftBorderFindFlag[i] = 'T';
        if (Img.RightBorder[i] <= 0 + lose_error
            || Img.RightBorder[i] >= COL - 1 - lose_error)
            Img.RightBorderFindFlag[i] = 'F', right_lost_cnt++;
        else
            Img.RightBorderFindFlag[i] = 'T';
    }

    //����·��
    int k_num = 3;
    float k_excel[COL];
    for (int i = 0; i < COL; i++)
        k_excel[i] = 1.0 * i / 10;
    for (int i = 0; i < ROW; i++) {
        if (i < 0 + k_num || i >= ROW - k_num)
            Img.RoadWide[i] = RoadWide0;
        else {
            if (Img.RightBorderFindFlag[i] == 'T'
                && Img.LeftBorderFindFlag[i] == 'T')
                Img.RoadWide[i] = RoadWide0;
            else if (Img.RightBorderFindFlag[i] == 'T'
                     && Img.RightBorderFindFlag[i + k_num] == 'T'
                     && Img.RightBorderFindFlag[i - k_num] == 'T'
                     && Img.LeftBorderFindFlag[i] == 'F')
                Img.RoadWide[i] =
                        RoadWide0
                        * (1
                           + k_excel[abs(
                                Img.RightBorder[i + k_num]
                                - Img.RightBorder[i - k_num])]);
            else if (Img.RightBorderFindFlag[i] == 'F'
                     && Img.LeftBorderFindFlag[i] == 'T'
                     && Img.LeftBorderFindFlag[i - k_num] == 'T'
                     && Img.LeftBorderFindFlag[i + k_num] == 'T')
                Img.RoadWide[i] = RoadWide0
                                  * (1
                                     + k_excel[abs(
                        Img.LeftBorder[i + k_num]
                        - Img.LeftBorder[i - k_num])]);
            else
                Img.RoadWide[i] = RoadWide0;
        }
    }
  //  edge_cnt(img_tmp);
  //  GetBorder( img_tmp);

   // solve_line(img_tmp);
}