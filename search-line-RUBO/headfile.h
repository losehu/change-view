
#ifndef MAIN_CPP_HEADFILE_H
#define MAIN_CPP_HEADFILE_H
typedef unsigned char       uint8;                                              // 无符号  8 bits
typedef unsigned short int  uint16;                                             // 无符号 16 bits
typedef unsigned long int   uint32;                                             // 无符号 32 bits
typedef unsigned long long  uint64;                                             // 无符号 64 bits
typedef unsigned char        uint8_t;
typedef char                int8;                                               // 有符号  8 bits
typedef short int           int16;                                              // 有符号 16 bits
typedef long  int           int32;                                              // 有符号 32 bits
typedef long  long          int64;                                              // 有符号 64 bits
#define COL  114
#define ROW  100
uint8 ImageUsed[ROW][COL];
#define StartScanRow    0//开始扫线行
typedef struct{
    int16 Center[ROW];//中
    int16 Center_Repair[ROW];//修复后的中线
    uint8 Center_Error[ROW];//图像是否连续标志

    int16 LeftBorder[ROW];//左
    int16 LeftBorder_Repair[ROW];
    uint8 LeftBorderFindFlag[ROW];
    uint8 LeftBorderDy[ROW];

    float LeftSlope[ROW];//斜率，存在边界时有效

    int16 RightBorder[ROW];//右
    int16 RightBorder_Repair[ROW];
    uint8 RightBorderFindFlag[ROW];
    uint8 RightBorderDy[ROW];

    float RightSlope[ROW];

    int8  RoadWide[ROW];//摄像头测得宽度
    int8  RoadWideError[ROW];//和赛道宽度表比较得到的误差
    uint8 RoadNormalFlag[ROW];//赛道宽度
}ImageDealType;
ImageDealType   Img;
#define  RoadWide0 45
/********八领域***********/
typedef struct  {
    int16 x0;
    int16 y0;
}size_point;
extern int16 seed_left_x,seed_left_y,seed_right_x,seed_right_y;
#define stack_size 300
#define stack_size1 100

extern size_point stack_seed2[stack_size1];    //栈
extern size_point stack_seed3[stack_size1];    //栈
extern int16 stack_top = 0,stack_top1 = 0,stack_top2=0,stack_top3=0;
extern   size_point stack_seed[stack_size];    //栈
extern size_point stack_seed1[stack_size];//栈
extern bool stop_flag;
;
extern int16 max_py,max_py1;
extern int16 min_py,min_py1;
bool pull_stack(int16 x, int16 y, uint8_t img_tmp[][COL]);    //入栈
size_point push_stack();    //出栈
bool pull_stack1(int16 x, int16 y, uint8_t p_Pixels[][COL]); //入栈
size_point push_stack1(); //出栈
void search_line(void);
extern int left_right_cnt,left_left_cnt,right_right_cnt,right_left_cnt;
void solve_line();
void cal_middle_line(void);
extern int16 start_line ;
extern int lose_error;
extern int16 left_lost_cnt , right_lost_cnt ;

extern int CenterCalMinRow_circle ;
extern int CenterCalMaxRow_circle ; //15


//连通域
typedef struct  {
    uint8_t edge[(ROW + COL) * 2 - 4];
    int room;
    int last_room;
    int room_cnt;
    int edge_len;
    int error_cnt,error_max;

}Connected_Component;
extern Connected_Component connect;
bool add_edge(int num, uint8_t img_tmp[ROW][COL]);
void init_connect();
void edge_cnt(uint8_t img_tmp[ROW][COL]);
void room_cnt();


float point_center0,point_center,point_center2,point_center1=0;
#endif