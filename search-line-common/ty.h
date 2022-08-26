//
// Created by rubo on 2022/1/2.
//

#ifndef MAIN_CPP_TY_H
#define MAIN_CPP_TY_H
#include<opencv2/opencv.hpp>
#include "headfile.h"
using namespace cv;
using namespace std;


Mat src;

void v_to_mat(Mat &image1, unsigned char b[ROW][COL]) { // 数组转矩阵
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            image1.at<uchar>(i, j) = b[i][j] * 255;
}
void mat_to_v(Mat &image1, unsigned char b[ROW][COL]) { // 数组转矩阵
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            b[i][j] = image1.at<uchar>(i, j);
}
void draw(int x, int y, int flag) {

    if (flag == 1) //B
    {
        src.at<Vec3b>(x, y)[0] = 255;
        src.at<Vec3b>(x, y)[1] = 0;
        src.at<Vec3b>(x, y)[2] = 0;
    } else if (flag == 2)//G
    {
        src.at<Vec3b>(x, y)[0] = 0;
        src.at<Vec3b>(x, y)[1] = 255;
        src.at<Vec3b>(x, y)[2] = 0;
    } else if (flag == 3) {
        src.at<Vec3b>(x, y)[0] = 0;
        src.at<Vec3b>(x, y)[1] = 0;
        src.at<Vec3b>(x, y)[2] = 255;
    }
    else if (flag == 4) {
        src.at<Vec3b>(x, y)[0] = 0;
        src.at<Vec3b>(x, y)[1] = 255;
        src.at<Vec3b>(x, y)[2] = 255;
    }
    else if(flag==5)
    {
        src.at<Vec3b>(x, y)[0] = 100;
        src.at<Vec3b>(x, y)[1] = 200;
        src.at<Vec3b>(x, y)[2] = 50;

    }
}
///************************八领域*******************************/
size_point stack_seed[stack_size];    //栈
size_point stack_seed1[stack_size];    //栈
int16 start_line = 10;
int16 left_lost_cnt = 0, right_lost_cnt = 0;
int16 stack_top = 0;
bool stop_flag = 0;
bool cam_die=0;
int16 stack_top1 = 0;
int16 max_py = 0, max_py1 = 0;
int16 min_py = 0, min_py1 = 0;
const int CenterCalMinRow_tmp = 0;
const int CenterCalMaxRow_tmp = 40;
int CenterCalMinRow = CenterCalMinRow_tmp;
int CenterCalMaxRow = CenterCalMaxRow_tmp; //15
int CenterCalMinRow_circle = 0;
int CenterCalMaxRow_circle = 20; //15

int lose_error = 10;
#define line_debug 1
bool search_line_flag = 0;
size_point connects[16] = {    //八领域扫点
        { 0, -1 },    //上
        { -1, -1 },    //左上123 567
        { -1, 0 },    //左
        { -1, 1 },    //左下
        { 0, 1 },    //下
        { 1, 1 },    //右下
        { 1, 0 },    //右
        { 1, -1 },    //右上
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
int left_right_cnt = 0, left_left_cnt = 0, right_right_cnt = 0, right_left_cnt =
        0;
int16  seed_left_x,seed_left_y,seed_right_x,seed_right_y;

void search_line(void) {
    left_right_cnt = 0, left_left_cnt = 0, right_right_cnt = 0, right_left_cnt =
            0;
    for (int i = 0; i < ROW; i++)
        Img.LeftBorder[i] = 0, Img.RightBorder[i] = COL - 1;
    int16 px, py;
    uint8_t img_tmp[ROW][COL];
    size_point center_seed;
    stack_top = 0;
    stack_top1 = 0;
    start_line = 10;
    int search_len = 20;
    int search_start = 0;
    int turn_flag = 0;
    seed_left_x = 0;
    seed_left_y = 0;
    seed_right_x = 0;
    seed_right_y = 0;
    init_connect();

    int search_start_sum = 0, search_start_sum_cnt = 0;
    for (int i = 0; i < COL; i++) {
        if (ImageUsed[ROW - 1][i] != 0) {
            search_start_sum += i;
            search_start_sum_cnt++;
        }
    }
    search_start = search_start_sum / search_start_sum_cnt;
    //  search_start = COL / 2 - 1;
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
                    || (ImageUsed[ROW - 1 - start_line][i] != 0 && i == 0)) {
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
             i <= COL - 1; i++) {
            if ((ImageUsed[ROW - 1 - start_line][i]
                 != ImageUsed[ROW - 1 - start_line][i - 1]
                 && ImageUsed[ROW - 1 - start_line][i] == 0)
                || (i == COL - 1 && ImageUsed[ROW - 1 - start_line][i] != 0)
                || (ImageUsed[ROW - 1 - start_line][i] != 0 && i == COL - 1)) {
                if (ImageUsed[ROW - 1 - start_line][i - 1]
                    == ImageUsed[ROW - 1 - start_line][i - 2]
                    || (ImageUsed[ROW - 1 - start_line][i] != 0
                        && i == COL - 1)) {
                    if (pull_stack1(i - 1, start_line, img_tmp))
                        return;
                    seed_right_x = i - 1;
                    seed_right_y = start_line;
                    break;
                }
            }
        }
        if (seed_right_x - seed_left_x <= 10 && turn_flag == 0) {//车库
            turn_flag = 1;
        }
        if (turn_flag == 0)
            start_line--;
        else if (turn_flag == 1) {
            if (seed_left_x != 0 && seed_right_x != 0) {
                start_line = start_line + 1, turn_flag++;
            } else {
                start_line = -120;
                break;
            }
        } else
            start_line++;
    }
//摄像头保护
    if (stack_top == 0 || stack_top1 == 0 || start_line < 0
        || search_line_flag == 1 || start_line > ROW - 1) {
        if (stop_flag == 0) {
            cout<<"cam die!"<<endl;
        }
        stop_flag = 1;
        return;
    } else cam_die = 0;
    max_py = 0, max_py1 = 0;
    min_py = 99, min_py1 = 99;


    while (stack_top + stack_top1 != 0 && !(max_py == max_py1 && max_py == 99)) {
        while (stack_top != 0 && (max_py <= max_py1 || stack_top1 == 0)) {
            center_seed = push_stack();
            for (int i = 0; i < 8; i++) {
                px = center_seed.x0 + connects[i].x0;
                py = center_seed.y0 + connects[i].y0;
                if (!(px < 0 || py < 0 || px >= COL || py >= ROW)) {
                    if (img_tmp[py][px] == 0) {
                        if (ImageUsed[ROW - 1 - py][px]
                            != ImageUsed[ROW - 1 - center_seed.y0][center_seed.x0]
                            || ((px == 0 || (py == 0 && i != 6))
                                && ImageUsed[ROW - 1 - py][px] != 0)) {
                            if (pull_stack(px, py, img_tmp))
                                return;

                            if (i == 1 || i == 2 || i == 3) //left
                                left_left_cnt++;
                            else if (i == 5 || i == 6 || i == 7)
                                left_right_cnt++;
                            if (ImageUsed[ROW - 1 - py][px] != 0)
                                while (px > 0
                                       && ImageUsed[ROW - 1 - py][px - 1] != 0)
                                    px--;
                            if (line_debug) {
                                draw(ROW-1-py,px,1);
                                                    imshow("result", src);
                    waitKey(1);
                            }
                            if (Img.LeftBorder[py] < px)
                                Img.LeftBorder[py] = px;
                            if (py > max_py)
                                max_py = py;
                        }
                    }
                }
            }
        }
        while (stack_top1 != 0 && (max_py >= max_py1 || stack_top == 0)) {
            center_seed = push_stack1();
            for (int i = 0; i < 8; i++) {
                px = center_seed.x0 + connects[i].x0;
                py = center_seed.y0 + connects[i].y0;
                if (!(px < 0 || py < 0 || px >= COL || py >= ROW)) {
                    if (img_tmp[py][px] == 0) {
                        if (ImageUsed[ROW - 1 - py][px]
                            != ImageUsed[ROW - 1 - center_seed.y0][center_seed.x0]
                            || ((px == COL - 1 || (py == 0 && i != 2))
                                && ImageUsed[ROW - 1 - py][px] != 0)) {
                            if (pull_stack1(px, py, img_tmp))
                                return;

                            if (i == 1 || i == 2 || i == 3)
                                right_left_cnt++;
                            else if (i == 5 || i == 6 || i == 7)
                                right_right_cnt++;
                            if (ImageUsed[ROW - 1 - py][px] != 0)
                                while (px < COL - 1
                                       && ImageUsed[ROW - 1 - py][px + 1] != 0)
                                    px++;
                            if (line_debug) {
                                draw(ROW-1-py,px,2);
                                imshow("result", src);
                                waitKey(1);

                            }
                            if (Img.RightBorder[py] > px)
                                Img.RightBorder[py] = px;
                            if (py > max_py1)
                                max_py1 = py;
                        }
                    }
                }
            }
        }
    }
    while (stack_top + stack_top1 != 0 && !(min_py == min_py1 && min_py == 0)) {
        while (stack_top != 0 && (stack_top1 == 0 || min_py >= min_py1)) {
            center_seed = push_stack();
            for (int i = 0; i < 8; i++) {
                px = center_seed.x0 + connects[i].x0;
                py = center_seed.y0 + connects[i].y0;
                if (!(px < 0 || py < 0 || px >= COL || py >= ROW)) {
                    if (img_tmp[py][px] == 0) {
                        if (ImageUsed[ROW - 1 - py][px]
                            != ImageUsed[ROW - 1 - center_seed.y0][center_seed.x0]
                            || ((px == 0 || (py == 0 && i != 6))
                                && ImageUsed[ROW - 1 - py][px] != 0)) {
                            if (pull_stack(px, py, img_tmp))
                                return;

                            if (i == 1 || i == 2 || i == 3) //left
                                left_left_cnt++;
                            else if (i == 5 || i == 6 || i == 7)
                                left_right_cnt++;
                            if (ImageUsed[ROW - 1 - py][px] != 0)
                                while (px > 0
                                       && ImageUsed[ROW - 1 - py][px - 1] != 0)

                                    px--;
                            if (line_debug) {
                                draw(ROW-1-py,px,1);
                                imshow("result", src);
                                waitKey(1);

                            }
                            if (Img.LeftBorder[py] < px)
                                Img.LeftBorder[py] = px;
                            if (py < min_py)
                                min_py = py;
                        }
                    }
                }
            }
        }
    }
    left_lost_cnt = 0, right_lost_cnt = 0;
    for (int i = 0; i < ROW; i++) {
        Img.RoadWide[i] = Img.RightBorder[i] - Img.LeftBorder[i];
        if (Img.LeftBorder[i] < 0 + lose_error
            || Img.LeftBorder[i] > COL - 1 - lose_error)
            Img.LeftBorderFindFlag[i] = 'F', left_lost_cnt++;
        else
            Img.LeftBorderFindFlag[i] = 'T';
        if (Img.RightBorder[i] < 0 + lose_error
            || Img.RightBorder[i] > COL - 1 - lose_error)
            Img.RightBorderFindFlag[i] = 'F', right_lost_cnt++;
        else
            Img.RightBorderFindFlag[i] = 'T';
    }

    edge_cnt(img_tmp);

// point_turn();
}

void cal_middle_line(void) {
//计算当前行的偏移量
//    float excursion[ROW];
    float sumcenter = 0;

    float cnt = 0;
    for ( int Ysite = CenterCalMinRow; Ysite < CenterCalMaxRow; Ysite++) {
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

//    if (circle_flag.enter_left_circle == 1 && circle_flag.on_left_circle == 0)
//    {
//        if (point_center0 > -point_center1)
//            point_center0 = point_center0 / 2;
//    }

}

void solve_line() {

    for (int Ysite = 0; Ysite < StartScanRow + 1; Ysite++) {
        Img.Center[Ysite] = COL / 2;
        Img.RightBorderFindFlag[Ysite] = 'T';
        Img.LeftBorderFindFlag[Ysite] = 'T';
    }
    /*****************正常************************/

    for (int Ysite = StartScanRow + 1; Ysite < ROW; Ysite++) {
        if (((Img.RightBorderFindFlag[Ysite] == 'T'
              && Img.LeftBorderFindFlag[Ysite] == 'F')
             || (Img.RightBorderFindFlag[Ysite] == 'F'
                 && Img.LeftBorderFindFlag[Ysite] == 'T'))
            && (Img.RightBorder[Ysite] - Img.LeftBorder[Ysite]
                >= RoadWide0 * 2)) {
            Img.LeftBorderFindFlag[Ysite] = 'F';
            Img.RightBorderFindFlag[Ysite] = 'F';
        }
        if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'T') //无丢线
            Img.Center[Ysite] = (Img.LeftBorder[Ysite] + Img.RightBorder[Ysite]) / 2;
        else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'F') //两边都丢
            Img.Center[Ysite] = COL/2;
        else if (Img.RightBorderFindFlag[Ysite] == 'F' && Img.LeftBorderFindFlag[Ysite] == 'T') //丢了右边线
            Img.Center[Ysite] = Img.LeftBorder[Ysite] + RoadWide0 / 2;
        else if (Img.RightBorderFindFlag[Ysite] == 'T' && Img.LeftBorderFindFlag[Ysite] == 'F')  //丢了左边
            Img.Center[Ysite] = Img.RightBorder[Ysite] - RoadWide0 / 2;

    }
    return;

}

/**************连通域*****************/
Connected_Component connect;
void init_connect() {
    memset(connect.edge, 0, sizeof(connect.edge));
    connect.room = 0;

    connect.edge_len = 3;
}
bool add_edge(int num, uint8_t img_tmp[ROW][COL]) {
    for (int j = 0; j < 2 * connect.edge_len + 1; j++) {
        int num_tmp = num - connect.edge_len + j;
        int i = (num_tmp + (COL + ROW) * 2 - 4) % ((COL + ROW) * 2 - 4);
        if (i < COL) {
            if (img_tmp[0][i] == 1)
                return 1;
            //  break;
        } else if (i < COL + ROW - 1) {
            if (img_tmp[i - (COL - 1)][COL - 1] == 1)
                return 1;
            //  break;

        } else if (i < COL + COL + ROW - 2) {
            if (img_tmp[ROW - 1][COL - 1 - i + (COL + ROW - 2)] == 1)
                return 1;
            //     break;

        } else {
            if (img_tmp[COL + COL + ROW + ROW - 4 - i][0] == 1)
                return 1;          //  break;

        }
    }
    return 0;
}
void edge_cnt(uint8_t img_tmp[ROW][COL]) {
    int x = 1, y = ROW - 1;
    int edge_cnt = 0;
    int way = 0; //0 r 1 up 2 l 3 down
    for (int i = 0; i < (COL + ROW) * 2 - 4; i++) {
        connect.edge[edge_cnt] = ImageUsed[y][x];
        connect.edge[edge_cnt] = ImageUsed[y][x];
        edge_cnt++;
        if (way == 0)
            x++;
        else if (way == 1)
            y--;
        else if (way == 2)
            x--;
        else if (way == 3)
            y++;
        if (x > COL - 1)
            way++, x = COL - 1;
        if (y < 0)
            way++, y = 0;
        if (x < 0)
            way++, x = 0;
        if (y > ROW - 1)
            way++, y = ROW - 1;
    }
    for (int i = 0; i < (COL + ROW) * 2 - 4; i++) {
        int sum_start = 0, sum_end = 0;
        for (int j = i; j < connect.edge_len * 2 + i; j++) {
            if (j < (COL + ROW) * 2 - 4) {
                if (j < i + connect.edge_len)
                    sum_start += connect.edge[j];
                else
                    sum_end += connect.edge[j];
            } else {
                if (j < i + connect.edge_len)
                    sum_start += connect.edge[j - ((COL + ROW) * 2 - 4)];
                else
                    sum_end += connect.edge[j - ((COL + ROW) * 2 - 4)];
            }
        }
        if (abs(sum_end - sum_start) == 255 * connect.edge_len
            && add_edge(i, img_tmp))
            connect.room++;
    }
    room_cnt() ;

}
void room_cnt() {
    if (connect.room == connect.last_room)
        connect.room_cnt++;
    else {
        connect.error_cnt++;
        if (connect.error_cnt >= connect.error_max) {
            connect.room_cnt = 1;
            connect.error_cnt=0;
            connect.last_room = connect.room;

        }
    }

}
#endif //MAIN_CPP_TY_H
