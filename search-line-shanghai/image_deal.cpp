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
#define line_debug 0
bool search_line_flag = 0;
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
int16 up_left_flag, up_right_flag, up_left_num, up_right_num;
int go[4][8][2] = {
        //     左,左前,前,右前,右,右后,后,左后                //方位/朝向
        {{-1, 0},  {-1, -1}, {0,  -1}, {1,  -1}, {1,  0},  {1,  1},  {0,  1},  {-1, 1}},//左
        {{0,  -1}, {1,  -1}, {1,  0},  {1,  1},  {0,  1},  {-1, 1},  {-1, 0},  {-1, -1}},//上
        {{1,  0},  {1,  1},  {0,  1},  {-1, 1},  {-1, 0},  {-1, -1}, {0,  -1}, {1,  -1}},//右
        {{0,  1},  {-1, 1},  {-1, 0},  {-1, -1}, {0,  -1}, {1,  -1}, {1,  0},  {1,  1}},//下
};
bool blocked(int head, int dir, int center_x, int center_y) {
    center_x = go[head][dir][1] + center_x;
    center_y = go[head][dir][0] + center_y;
    if (center_x < 0 || center_x >= COL)return 1;
    if (center_y < 0 || center_y >= ROW)return 1;
    if (ImageUsed[ROW - 1 - center_y][center_x] == 0)return 1;
    return 0;
}
void search_line(void) {
    for (int i = 0; i < ROW; i++)
        Img.LeftBorder[i] = 0, Img.RightBorder[i] = COL - 1;
    uint8_t img_tmp[ROW][COL];
    stack_top = 0, stack_top1 = 0;
    start_line = 5;
    int search_len = 20;
    int search_start = 0;
    int turn_flag = 0;
    seed_left_x = 0, seed_left_y = 0;
    seed_right_x = 0, seed_right_y = 0;
    init_connect();
    bool down_flag = 0;
    int search_start_sum = 0, search_start_sum_cnt = 0;
    for (int i = 0; i < COL; i++) {
        if (ImageUsed[ROW - 1][i] != 0) {
            search_start_sum += i;
            search_start_sum_cnt++;
        }
    }
    int left_step[300][2], right_step[300][2], left_step_cnt = 0, right_step_cnt = 0;
    int sum_step[500][2];
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
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i + 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0)
                || (i == 0 && ImageUsed[ROW - 1 - start_line][i] != 0)
                || (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0)) {
                if (ImageUsed[ROW - 1 - start_line][i + 1]
                    == ImageUsed[ROW - 1 - start_line][i + 2]
                        ) {
                    if (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0) {
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
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i - 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0)
                || (i == COL - 1 && ImageUsed[ROW - 1 - start_line][i] != 0)
                || (ImageUsed[ROW - 1 - start_line][i] != 0 && i == COL - 1)) {
                if (ImageUsed[ROW - 1 - start_line][i] != 0
                    && i == COL - 1) {
                    if (pull_stack1(i, start_line, img_tmp))
                        return;
                    seed_right_x = i;
                    seed_right_y = start_line;
                    break;
                } else if (ImageUsed[ROW - 1 - start_line][i - 1]
                           == ImageUsed[ROW - 1 - start_line][i - 2]
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
                start_line = 5 + 10, turn_flag++;
            } else {
                start_line = -120;
                break;
            }
        } else start_line++;
    }
    if (stack_top == 0 || stack_top1 == 0 || start_line < 0 //摄像头保护
        || search_line_flag == 1 || start_line > ROW - 1) {
        ExSpeed = 0;
        if (stop_flag == 0) {
            ips114_clear(GREEN);
        }
        stop_flag = 1;
        gpio_low(BEEP_PIN);
        ips114_show_string(1, 1, "speed die!");
        return;
    } else
        cam_die = 0;
    up_left_flag = 0, up_right_flag = 0;
    up_left_num = 0, up_right_num = 0;
    if (line_debug) {
        ips114_clear(YELLOW);
        my_delay(10);
    }
    int stop_y, stop_y1;
    int head_left = 0, head_right = 2;
    int left_center_x = seed_left_x, left_center_y = seed_left_y;
    int right_center_x = seed_right_x, right_center_y = seed_right_y;
    Img.LeftBorder[left_center_y] = left_center_x;
    Img.RightBorder[right_center_y] = right_center_x;
    max_py = left_center_y, max_py1 = right_center_y;
    min_py = left_center_y, min_py1 = right_center_y;
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
            while (blocked(head_left, 2, left_center_x, left_center_y))head_left++, head_left = (head_left + 4) % 4;
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
                head_left--;
                head_left = (head_left + 4) % 4;
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
            if (left_center_x == 0)stop_y = left_center_y;
            //else if(min_py>left_center_y)   min_py=left_center_y;
            if (min_py1 > left_center_y && down_flag) {
                min_py1 = left_center_y;
                if (min_py == min_py1 && min_py == 0) {
                    if (Img.RightBorder[left_center_y] > left_center_x)
                        Img.RightBorder[left_center_y] = left_center_x;
                    if (Img.LeftBorder[right_center_y] < right_center_x)
                        Img.LeftBorder[right_center_y] = right_center_x;
                    img_tmp[left_center_y][left_center_x] = 1;
                    break;
                }
            }
            img_tmp[left_center_y][left_center_x] = 1;
            if (!down_flag) {
                if (Img.LeftBorder[left_center_y] < left_center_x && !up_left_flag)
                    Img.LeftBorder[left_center_y] = left_center_x;
                if (line_debug) {
                    ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 3);
                    my_delay(10);
                }
            } else {
                if (Img.RightBorder[left_center_y] > left_center_x)
                    Img.RightBorder[left_center_y] = left_center_x;
                if (line_debug) {
                    ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 1);
                    my_delay(10);
                }
            }
        }
        if (((max_py1 <= max_py && !up_right_flag) || (up_right_flag && up_left_num >= up_right_num)) && min_py != 0 ||
            ((max_py1 <= max_py && !up_right_flag) || (up_right_flag && up_left_num >= up_right_num)) && !down_flag
            || down_flag == 1 && min_py != 0) {
            while (blocked(head_right, 2, right_center_x, right_center_y))
                head_right--, head_right = (head_right + 4) % 4;
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
                head_right++;
                head_right = (head_right + 4) % 4;
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
            if (right_center_x == 0)stop_y1 = right_center_y;

            //else if(min_py1>seed_right_y) min_py1=seed_right_y;
            if (min_py > right_center_y && down_flag) {
                min_py = right_center_y;
                if (min_py == min_py1 && min_py1 == 0) {
                    if (Img.RightBorder[left_center_y] > left_center_x)
                        Img.RightBorder[left_center_y] = left_center_x;
                    if (Img.LeftBorder[right_center_y] < right_center_x)
                        Img.LeftBorder[right_center_y] = right_center_x;
                    img_tmp[right_center_y][right_center_x] = 1;
                    break;
                }
            }
            img_tmp[right_center_y][right_center_x] = 1;
            if (!down_flag) {
                if (Img.RightBorder[right_center_y] > right_center_x && !up_right_flag)
                    Img.RightBorder[right_center_y] = right_center_x;
                if (line_debug) {
                    ips114_draw_point(right_center_x, ROW - 1 - right_center_y, 1);
                    my_delay(10);
                }
            } else {
                if (Img.LeftBorder[right_center_y] < right_center_x)
                    Img.LeftBorder[right_center_y] = right_center_x;
                if (line_debug) {
                    ips114_draw_point(right_center_x, ROW - 1 - right_center_y, 3);
                    my_delay(10);
                }
            }
        }
    }
    left_lost_cnt = 0, right_lost_cnt = 0;
    for (int i = 0; i < ROW; i++) {
        if (fork_flag.on_fork == 0
            || fork_flag.on_fork == 1 && fork_flag.out_fork == 1) {
            if (i < stop_y) {
                if (Img.LeftBorder[i] <= 0 + lose_error
                    || Img.LeftBorder[i] >= COL - 1 - lose_error)
                    Img.LeftBorderFindFlag[i] = 'F', left_lost_cnt++;
                else
                    Img.LeftBorderFindFlag[i] = 'T';
            } else {
                Img.LeftBorderFindFlag[i] = 'F';
            }
            if (i < stop_y1) {
                if (Img.RightBorder[i] <= 0 + lose_error
                    || Img.RightBorder[i] >= COL - 1 - lose_error)
                    Img.RightBorderFindFlag[i] = 'F', right_lost_cnt++;
                else
                    Img.RightBorderFindFlag[i] = 'T';
            } else {
                Img.RightBorderFindFlag[i] = 'F';
            }
        } else {
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
    }
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
    edge_cnt(img_tmp);
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
typedef struct {
    int x1, y1, x2, y2, x3, y3;
    int fork_step[600][2], fork_step_cnt = 0;
    int cal_len = 8;
    int room_cnt = 0;
    int point1_start = 0, point2_start = 0, point3_start = 0, point_len = 8;
    int point1_end = 0, point2_end = 0, point3_end = 0;
    int point1[10][2], point2[10][2], point3[10][3], point1_cnt = 0, point2_cnt = 0, point3_cnt = 0;
    int point2_tmp=999,point1_tmp=999,point3_tmp=999;
    int x_left=999,x_right=999;
} fork_fix_flag;
fork_fix_flag fork_fix;
bool judge_edge(int x, int y) {
    if (x == 0 || x == COL - 1 || y == 0 || y == ROW - 1)return 1;
    return 0;
}
int connect_line(int x1 ,int y1,int x2,int y2)
{
    int i=0;
    float k,b;
    int y_max,y_min;
    int x_max,x_min;
    y_max = max_ab(y1,y2);
    y_min = min_ab(y1,y2);
    x_max = max_ab(x1,x2);
    x_min = min_ab(x1,x2);
    k = 1.0*(y2-y1)/(x2 - x1); //直线
    b = 1.0*(((y2+y1)-k*(x2+x1)))/2;
    for(i= y_min; i<= y_max; i++)
    {
               ips114_draw_point((int)(1.0*(i-b)/k),ROW-1-i,8);
    }
}
void init_fix_fork()
{
    fork_fix.fork_step_cnt = 0;
    fork_fix.  room_cnt = 0;
    fork_fix. point_len = 8;
    fork_fix. cal_len = 8;

    fork_fix.  point1_start = 0,fork_fix. point2_start = 0, fork_fix.point3_start = 0;
    fork_fix.  point1_cnt = 0,fork_fix. point2_cnt = 0,fork_fix. point3_cnt = 0;
    fork_fix.    point2_tmp=999,fork_fix.point1_tmp=999,fork_fix.point3_tmp=999;
    fork_fix.  x_left=999,fork_fix.x_right=999;

}
void fork_solve() {

    int room_flag = 0;
    for (int i = 2 * fork_fix.cal_len + 1; i < fork_fix.fork_step_cnt; i++) {
        bool useful_flag = 1;
        if (judge_edge(fork_fix.fork_step[i][0], fork_fix.fork_step[i][1]) ==
            judge_edge(fork_fix.fork_step[(i + 1)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 1)%fork_fix.fork_step_cnt][1]) &&
            judge_edge(fork_fix.fork_step[(i + 1)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 1)%fork_fix.fork_step_cnt][1]) ==
            judge_edge(fork_fix.fork_step[(i + 2)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 2)%fork_fix.fork_step_cnt][1]) &&
            judge_edge(fork_fix.fork_step[(i + 3)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 3)%fork_fix.fork_step_cnt][1]) ==
            judge_edge(fork_fix.fork_step[(i + 4)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 4)%fork_fix.fork_step_cnt][1]) &&
            judge_edge(fork_fix.fork_step[(i + 4)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 4)%fork_fix.fork_step_cnt][1]) ==
            judge_edge(fork_fix.fork_step[(i + 5)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 5)%fork_fix.fork_step_cnt][1]) &&
            judge_edge(fork_fix.fork_step[(i + 3)%fork_fix.fork_step_cnt][0], fork_fix.fork_step[(i + 3)%fork_fix.fork_step_cnt][1]) !=
            judge_edge(fork_fix.fork_step[i][0], fork_fix.fork_step[i][1]))
            room_flag++;
        if(room_flag==5&& fork_fix.x_right==999)  fork_fix.x_right=fork_fix.fork_step[(i+3)%fork_fix.fork_step_cnt][0];
        if(room_flag==6&& fork_fix.x_left==999) fork_fix.x_left=fork_fix.fork_step[(i+3)%fork_fix.fork_step_cnt][0];

        for (int j = i - fork_fix.cal_len; j <= i + fork_fix.cal_len; j++) {
            if (fork_fix.fork_step[j][0] == 0 || fork_fix.fork_step[j][0] == COL - 1 || fork_fix.fork_step[j][1] == 0 ||
                fork_fix.fork_step[j][1] == ROW - 1) {
                useful_flag = 0;
                break;
            }
        }
        if (useful_flag == 0)continue;
        float x1 = fork_fix.fork_step[i - fork_fix.cal_len][0], y1 = fork_fix.fork_step[i - fork_fix.cal_len][1];
        float x2 = fork_fix.fork_step[i][0], y2 = fork_fix.fork_step[i][1];
        float x3 = fork_fix.fork_step[i + fork_fix.cal_len][0], y3 = fork_fix.fork_step[i + fork_fix.cal_len][1];
        float cos = ((x1 - x2) * (x3 - x2) + (y1 - y2) * (y3 - y2)) /
                    (sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2)) *
                     sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        if (cos <= -0.35 && cos >= -0.65) {
     //       ips114_draw_point(x2,ROW-1-y2,2);
            //    cout<<    room_flag<<endl;

            if (room_flag == 0) {
                if (fork_fix.point1_start == 0)fork_fix.point1_start = i;
                else {
                    if (i < fork_fix.point_len + fork_fix.point1_start)fork_fix.point1_end = i;
                }
            }
            if (room_flag == 2) {
                if (fork_fix.point2_start == 0)fork_fix.point2_start = i;
                else {
                    if (i < fork_fix.point_len + fork_fix.point2_start)fork_fix.point2_end = i;
                }
            }
            if (room_flag == 4) {
                if (fork_fix.point3_start == 0)fork_fix.point3_start = i;
                else {
                    if (i < fork_fix.point_len + fork_fix.point3_start)fork_fix.point3_end = i;
                }
            }
        }
        if (i >= fork_fix.point_len + fork_fix.point1_start && fork_fix.point1_start != 0) {
            fork_fix.x1 = fork_fix.fork_step[
                    (fork_fix.point1_start + (fork_fix.point1_end == 0 ? fork_fix.point1_start : fork_fix.point1_end)) /
                    2][0];
            fork_fix.y1 = fork_fix.fork_step[
                    (fork_fix.point1_start + (fork_fix.point1_end == 0 ? fork_fix.point1_start : fork_fix.point1_end)) /
                    2][1];
            fork_fix.point1[fork_fix.point1_cnt][0] = fork_fix.x1;
            fork_fix.point1[fork_fix.point1_cnt][1] = fork_fix.y1;
            fork_fix.point1_cnt++;
            fork_fix.point1_start = 0;
            fork_fix.point1_end = 0;
        }
        if (i >= fork_fix.point_len + fork_fix.point2_start && fork_fix.point2_start != 0) {
            fork_fix.x2 = fork_fix.fork_step[
                    (fork_fix.point2_start + (fork_fix.point2_end == 0 ? fork_fix.point2_start : fork_fix.point2_end)) /
                    2][0];
            fork_fix.y2 = fork_fix.fork_step[
                    (fork_fix.point2_start + (fork_fix.point2_end == 0 ? fork_fix.point2_start : fork_fix.point2_end)) /
                    2][1];
            fork_fix.point2[fork_fix.point2_cnt][0] = fork_fix.x2;
            fork_fix.point2[fork_fix.point2_cnt][1] = fork_fix.y2;
            fork_fix.point2_cnt++;
            fork_fix.point2_start = 0;
            fork_fix.point2_end = 0;
        }
        if (i >= fork_fix.point_len + fork_fix.point3_start && fork_fix.point3_start != 0) {
            fork_fix.x3 = fork_fix.fork_step[
                    (fork_fix.point3_start + (fork_fix.point3_end == 0 ? fork_fix.point3_start : fork_fix.point3_end)) /
                    2][0];
            fork_fix.y3 = fork_fix.fork_step[
                    (fork_fix.point3_start + (fork_fix.point3_end == 0 ? fork_fix.point3_start : fork_fix.point3_end)) /
                    2][1];
            fork_fix.point3[fork_fix.point3_cnt][0] = fork_fix.x3;
            fork_fix.point3[fork_fix.point3_cnt][1] = fork_fix.y3;
            fork_fix.point3_cnt++;
            fork_fix.point3_start = 0;
            fork_fix.point3_end = 0;
        }
    }
if( fork_fix.point2_cnt==0)return;
    for (int i = 0; i < fork_fix.point2_cnt; i++) {
        if (abs(fork_fix.point2[i][0] - COL / 2) < fork_fix.point2_tmp) {
            fork_fix.point2_tmp = abs(fork_fix.point2[i][0] - COL / 2);
            fork_fix.x2 = fork_fix.point2[i][0];
            fork_fix.y2 = fork_fix.point2[i][1];
        }
    }
    if(fork_fix.point1_cnt==1)fork_fix.x1=fork_fix.point1[0][0],fork_fix.y1=fork_fix.point1[0][1];
    else if(fork_fix.point1_cnt>1)
    {
        for (int i = 0; i < fork_fix.point1_cnt; i++) {
            if (abs(sqrt(1.0*(fork_fix.x2-fork_fix.point1[i][0])*(fork_fix.x2-fork_fix.point1[i][0])+(fork_fix.y2-fork_fix.point1[i][1])*(fork_fix.y2-fork_fix.point1[i][1]))-45)< fork_fix.point1_tmp) {
                fork_fix.point1_tmp = abs(sqrt(1.0*(fork_fix.x2-fork_fix.point1[i][0])*(fork_fix.x2-fork_fix.point1[i][0])+(fork_fix.y2-fork_fix.point1[i][1])*(fork_fix.y2-fork_fix.point1[i][1]))-45);
                fork_fix.x1 = fork_fix.point1[i][0];
                fork_fix.y1 = fork_fix.point1[i][1];
            }
        }
    }

    if(fork_fix.point3_cnt==1)fork_fix.x3=fork_fix.point3[0][0],fork_fix.y3=fork_fix.point3[0][1];
    else if(fork_fix.point3_cnt>1)
    {
        for (int i = 0; i < fork_fix.point3_cnt; i++) {
            if (abs(sqrt(1.0*(fork_fix.x2-fork_fix.point3[i][0])*(fork_fix.x2-fork_fix.point3[i][0])+(fork_fix.y2-fork_fix.point3[i][1])*(fork_fix.y2-fork_fix.point3[i][1]))-45)< fork_fix.point3_tmp) {
                fork_fix.point3_tmp = abs(sqrt(1.0*(fork_fix.x2-fork_fix.point3[i][0])*(fork_fix.x2-fork_fix.point3[i][0])+(fork_fix.y2-fork_fix.point3[i][1])*(fork_fix.y2-fork_fix.point3[i][1]))-45);
                fork_fix.x3 = fork_fix.point3[i][0];
                fork_fix.y3 = fork_fix.point3[i][1];
            }
        }
    }


if(fork_fix.point1_cnt==0)
{
    cout<<"OK"<<endl;
    fork_fix.x1=fork_fix.x_left;
    fork_fix.y1=0;


}

if(fork_fix.point3_cnt==0)
{
    fork_fix.x3=fork_fix.x_right;
    fork_fix.y3=0;
}


   connect_line(fork_fix.x1,fork_fix.y1,fork_fix.x2,fork_fix.y2);
   connect_line(fork_fix.x3,fork_fix.y3,fork_fix.x2,fork_fix.y2);
}
void fix_frok_search(void) {
    init_fix_fork();
    uint8_t img_tmp[ROW][COL];
    start_line = 1;
    int search_len = 20;
    int search_start = 0;
    seed_left_x = 0, seed_left_y = 0;
    int search_start_sum = 0, search_start_sum_cnt = 0;
    for (int i = 0; i < COL; i++) {
        if (ImageUsed[ROW - 1][i] != 0) {
            search_start_sum += i;
            search_start_sum_cnt++;
        }
    }
    search_start = search_start_sum / search_start_sum_cnt;
    memset(img_tmp, 0, sizeof(img_tmp));
    while (start_line >= 0) {
        stack_top = 0, stack_top1 = 0;
        seed_right_x = 0, seed_left_x = 0;
        for (int i =
                (search_start + search_len) < COL - 3 ?
                (search_start + search_len) : COL - 3; i >= 0; i--) {
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i + 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0)
                || (i == 0 && ImageUsed[ROW - 1 - start_line][i] != 0)
                || (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0)) {
                if (ImageUsed[ROW - 1 - start_line][i + 1]
                    == ImageUsed[ROW - 1 - start_line][i + 2]
                        ) {
                    if (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0) {
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
        start_line--;
    }
    if (line_debug) {
        ips114_draw_point(seed_left_x, ROW - 1 - seed_left_y, 1);
        my_delay(10);
    }
    fork_fix.fork_step[fork_fix.fork_step_cnt][0] = seed_left_x;
    fork_fix.fork_step[fork_fix.fork_step_cnt][1] = seed_left_y;
    fork_fix.fork_step_cnt++;
    if (stack_top == 0)return;
    int head_left = 0;
    int left_center_x = seed_left_x, left_center_y = seed_left_y;
    max_py = left_center_y;
    min_py = left_center_y;
    bool search_flag = 1;
    //head

    int buff_cnt = 0;
    while (search_flag) {
        while (blocked(head_left, 2, left_center_x, left_center_y))head_left++, head_left = (head_left + 4) % 4;
        if (!blocked(head_left, 1, left_center_x, left_center_y)) {
            left_center_x += go[head_left][1][1];
            left_center_y += go[head_left][1][0];
            buff_cnt++;
            fork_fix.fork_step[fork_fix.fork_step_cnt][0] = left_center_x;
            fork_fix.fork_step[fork_fix.fork_step_cnt][1] = left_center_y;
            fork_fix.fork_step_cnt++;
            if (line_debug) {
                ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 1);
                my_delay(10);
            }
            head_left--;
            head_left = (head_left + 4) % 4;
        } else if (blocked(head_left, 1, left_center_x, left_center_y)) {
            left_center_x += go[head_left][2][1];
            left_center_y += go[head_left][2][0];
            buff_cnt++;
            fork_fix.fork_step[fork_fix.fork_step_cnt][0] = left_center_x;
            fork_fix.fork_step[fork_fix.fork_step_cnt][1] = left_center_y;
            fork_fix.fork_step_cnt++;
            if (line_debug) {
                ips114_draw_point(left_center_x, ROW - 1 - left_center_y, 1);
                my_delay(10);
            }
        }
        img_tmp[left_center_y][left_center_x] = 1;
        if (left_center_x == seed_left_x && seed_left_y == left_center_y)search_flag = 0;
    }
    //end
    fork_solve();
}

