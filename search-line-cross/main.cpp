#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include "zf_common_headfile.h"
using namespace cv;
using namespace std;
Mat src;
void mouse_call_back(int event, int mouse_x, int mouse_y, int flags, void *userdata);
int main() {
    init_flag();
    fork_flag.cnt = 0;
    init_img("C:\\Users\\Administrator\\Desktop\\cross2.png");
    //flip(src, src, 1);
    uint8_t a[ROW][COL];
    for(int i=5;i<15;i++)
    {
        for(int j=COL/2-10;j<COL/2+10;j++)
        {
        //    src.at<unsigned char>(ROW-1-i,j)=0;
        }
    }
    mat_to_v(src, a);
    uint8_t img[ROW][COL];
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)PerImg_ip[i][j] = &a[i][j];
    cvtColor(src, src, COLOR_GRAY2RGB);
//    search_line();
//    judge_unit();
//    GetBorder();
//    solve_line();
//    cal_middle_line();
    fix_frok_search();
    cout << "连通域: " << connect.room << endl;
    for (int i = ROW - 1; i >= 0; i--) {
        if ( ImageUsed[i][Img.LeftBorder[ROW - 1 - i]] != 0)
            draw(i,  Img.LeftBorder[  ROW - 1 -  i],6);
        if ( ImageUsed[i][Img.RightBorder[ROW - 1 - i]] != 0)
            draw(i,Img.RightBorder[ ROW -1 -i],    4);
       draw(i, Img.Center[ROW - 1 - i], 5);
        waitKey(1);
        imshow("result", src);
    }
    setMouseCallback("result", mouse_call_back, 0);
    waitKey(0);
}
void mouse_call_back(int event, int mouse_x, int mouse_y, int flags, void *userdata) {
    switch (event) {
        case EVENT_RBUTTONDOWN: {
            cout << mouse_x << '\t' << mouse_y << endl;
        }
        case EVENT_LBUTTONUP: {      //鼠标获取坐标
            cout << Img.LeftBorderFindFlag[ROW - 1 - mouse_y] << '\t' << Img.RightBorderFindFlag[ROW - 1 - mouse_y]
                 << endl;
            cout << Img.LeftBorder[ROW - 1 - mouse_y] << '\t' << Img.RightBorder[ROW - 1 - mouse_y] << endl;
        }
    }
}



void judge_circle_left() {
    static int search_col = 0 + 10;

    if (circle_flag.find_left_circle == 0) {

        temp_flag = 0;
        if (connect.room < 4)
            return;
        if ((abs(left_lost_cnt - right_lost_cnt) < ROW / 2)
                && right_lost_cnt >= ROW / 5)
            return;
        if (left_lost_cnt < ROW / 3)
            return;
        if (!(ImageUsed[ROW - 1][search_col] == ImageUsed[ROW - 2][search_col]
                && ImageUsed[ROW - 1][search_col]
                        == ImageUsed[ROW - 1 - 5][search_col]
                && ImageUsed[ROW - 1][search_col] != 0
                && Img.RightBorderFindFlag[0] == Img.RightBorderFindFlag[1]
                && Img.RightBorderFindFlag[5] == Img.RightBorderFindFlag[5]
                && Img.RightBorderFindFlag[0] == 'T'))
            return;
        if (Img.RightBorderFindFlag[ROW / 4] == 'F'
                || Img.RightBorderFindFlag[ROW / 2] == 'F'
                || Img.RightBorderFindFlag[ROW / 4 * 3] == 'F')
            return;
        int block_cnt = 0;
        for (int i = ROW - 1; i >= ROW - 5; i--) {
            for (int j = search_col + 0; j < search_col + 5; j++) {
                if (ImageUsed[i][j] != 0)
                    block_cnt++;
            }
        }
        if (block_cnt <= 20)
            return;
        block_cnt = 0;
        for (int i = ROW - 1 - ROW / 2 + 2; i >= ROW - 1 - ROW / 2 - 2; i--) {
            for (int j = search_col + 0; j < search_col + 5; j++) {
                if (ImageUsed[i][j] == 0)
                    block_cnt++;
            }
        }
        if (block_cnt <= 20)
            return;

        block_cnt = 0; //右上角不能是白色！！
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                if (ImageUsed[ROW - 1 - i][COL - 1 - j] == 0)
                    block_cnt++;
            }
        }
        if (block_cnt <= 20)
            return;
        ips114_clear(PINK);

        int16 k_error = 4;
        int16 error_num0 = abs(
                Img.RightBorder[ROW - 1 - 0]
                        - Img.RightBorder[ROW - 1 - ROW / 4]);
        int16 error_num1 = abs(
                Img.RightBorder[ROW - 1 - ROW / 2]
                        - Img.RightBorder[ROW - 1 - ROW / 4]);
        int16 error_num2 = abs(
                Img.RightBorder[ROW - 1 - ROW / 2]
                        - Img.RightBorder[ROW - 1 - ROW / 4 * 3]);
        if (abs(error_num0 - error_num1) > k_error)
            return;
        if (abs(error_num1 - error_num2) > k_error)
            return;
        if (abs(error_num0 - error_num2) > 2 * k_error)
            return;
        error_num0 = abs(
                Img.RightBorder[ROW - 1 - 0]
                        - Img.RightBorder[ROW - 1 - ROW / 20]);
        error_num1 = abs(
                Img.RightBorder[ROW - 1 - ROW / 10]
                        - Img.RightBorder[ROW - 1 - ROW / 20]);
        error_num2 = abs(
                Img.RightBorder[ROW - 1 - ROW / 10]
                        - Img.RightBorder[ROW - 1 - ROW / 20 * 3]);
        if (abs(error_num0 - error_num1) > k_error)
            return;
        if (abs(error_num1 - error_num2) > k_error)
            return;
        if (abs(error_num0 - error_num2) > 2 * k_error)
            return;
        if (Img.RightBorder[0] - Img.LeftBorder[0] < RoadWide0 / 4 * 3)
            return;
        if (Img.RightBorder[1] - Img.LeftBorder[1] < RoadWide0 / 4 * 3)
            return;
        if (Img.RightBorder[5] - Img.LeftBorder[5] < RoadWide0 / 4 * 3)
            return;
        int sum_temp = 0;
        for (int i = 0; i < ROW; i++) {
            sum_temp += Img.RightBorder[i];
        }
        if (sum_temp < ROW * COL * 0.4)
            return;

        gpio_high(BEEP_PIN);
        close_judge(-1);

        circle_flag.find_left_circle = 1;
        ExSpeed_SUM = circle_speed;
        CenterCalMinRow = CenterCalMinRow_tmp;
        CenterCalMaxRow = CenterCalMaxRow_tmp;
        cntt++;
        ips114_clear(BROWN);
        kp1_tmp = ServoPIDParam[0][0];
        ServoPIDParam[0][0] = 2.5;
    } else if (circle_flag.enter_left_circle == 0) {
        if ((abs(left_lost_cnt - right_lost_cnt) < ROW / 3 * 2)
                && right_lost_cnt >= ROW / 5)
            return;
        if (!(Img.RightBorderFindFlag[0] == Img.RightBorderFindFlag[1]
                && Img.RightBorderFindFlag[5] == Img.RightBorderFindFlag[5]
                && Img.RightBorderFindFlag[0] == 'T'))
            return;
        if (Img.RightBorderFindFlag[ROW / 4] == 'F'
                || Img.RightBorderFindFlag[ROW / 2] == 'F'
                || Img.RightBorderFindFlag[ROW / 4 * 3] == 'F')
            return;
        int block_cnt = 0;
        for (int i = ROW - 1; i >= ROW - 10; i--) {
            for (int j = search_col + 0; j < search_col + 10; j++) {
                if (ImageUsed[i][j] == 0)
                    block_cnt++;
            }
        }
        if (block_cnt <= 80)
            return;
        block_cnt = 0;
        for (int i = ROW / 2 - 5; i <= ROW / 2 + 4; i++) {
            for (int j = search_col + 0; j < search_col + 10; j++) {
                if (ImageUsed[i][j] != 0)
                    block_cnt++;
            }
        }
        if (block_cnt <= 80)
            return;

        if (Img.RightBorder[0] - Img.LeftBorder[0] < RoadWide0 / 4 * 3)
            return;
        if (Img.RightBorder[1] - Img.LeftBorder[1] < RoadWide0 / 4 * 3)
            return;
        if (Img.RightBorder[5] - Img.LeftBorder[5] < RoadWide0 / 4 * 3)
            return;
        int sum_temp = 0;
        for (int i = 1; i < ROW / 2; i++) {
            if (Img.RightBorder[i] - Img.LeftBorder[i]
                    > Img.RightBorder[i - 1] - Img.LeftBorder[i - 1])
                sum_temp++;
        }
        if (sum_temp < 18)
            return;
        circle_flag.enter_left_circle = 1;

    } else if (circle_flag.on_left_circle == 0) {
        if (max_py1 >= ROW * 0.8)
            return;
        if (connect.room != 4)
            return;
        if (Img.RightBorder[0] - Img.RightBorder[ROW / 20] <= 0)
            return;
        if (Img.RightBorder[ROW / 20] - Img.RightBorder[ROW / 10] <= 0)
            return;
        if (Img.RightBorderFindFlag[max_py1 - 2] == 'F')
            return;
        if (Img.RightBorderFindFlag[max_py1 - 3] == 'F')
            return;
        if (Img.RightBorderFindFlag[max_py1 - 5] == 'F')
            return;
        circle_flag.on_left_circle = 1;

    } else if (circle_flag.out_left_circle == 0) {
        if (!(max_py >= ROW * 0.8 && max_py1 >= 0.8))
            return;
        circle_flag.out_left_circle = 1;

    } else if (circle_flag.leave_left_circle == 0) {
        if (temp_flag == 0) {
            if (max_py1 <= ROW * 0.9 || max_py <= ROW * 0.9)
                return;
            if (right_lost_cnt >= ROW * 0.1)
                return;

            temp_flag = 1;
            //   ExSpeed_SUM = circle_speed;

            //  ExSpeed_SUM = circle_speed - 100;

        } else {

//            if (max_py1 >=CenterCalMaxRow_circle*2 || max_py >=CenterCalMaxRow_circle*2)    return;
            int sum_temp = 0;
            for (int i = 0; i < ROW / 2; i++)
                sum_temp += Img.LeftBorder[i];
            if (sum_temp > ROW / 2 * 30)
                return;
            int useful = 0;
            for (int i = 0; i < ROW; i++) {
                if (Img.LeftBorderFindFlag[i] == 'F'
                        && Img.RightBorderFindFlag[i] == 'T')
                    useful++;
            }
            if (useful < ROW / 3 - 5)
                return;
            if (connect.room > 6)
                return;
            int block_cnt = 0;
            block_cnt = 0;

            for (int i = ROW / 5 + 0; i < ROW / 5 + 5; i++) {
                for (int j = COL - 1; j >= COL - 1 - 5; j--) {
                    if (ImageUsed[ROW - 1 - i][j] == 0)
                        block_cnt++;
                }
            }
            if (block_cnt <= 20)
                return;
            block_cnt = 0;
            for (int i = 0; i < max_py1 - 10; i++) {
                if (Img.RightBorder[i] > Img.RightBorder[i + 5])
                    block_cnt++;
            }

            if (block_cnt < max_py1 - 20)
                return;

            circle_flag.leave_left_circle = 1;

            gpio_low(BEEP_PIN);
            ExSpeed_SUM = circle_speed - 10;
            //  ServoPIDParam[0][0]=3.5;
            //    stop_flag=1;
            ServoPIDParam[0][0] = 1.0;

        }
    } else {
        if (Img.LeftBorderFindFlag[0] == 'F'
                || Img.RightBorderFindFlag[0] == 'F')
            return;
        int find_cnt_left = 0, find_cnt_right = 0, diff_cnt = 0;
        for (int i = 0; i < ROW / 4; i++) {
            if (Img.LeftBorderFindFlag[i] == 'T')
                find_cnt_left++;
            if (Img.RightBorderFindFlag[i] == 'T')
                find_cnt_right++;
            if (Img.RightBorder[i] - Img.LeftBorder[i] <= RoadWide0 + 10)
                diff_cnt++;
        }
        if (find_cnt_left < ROW / 4 - 2)
            return;
        if (find_cnt_right < ROW / 4 - 2)
            return;
        if (diff_cnt < ROW / 4 - 2)
            return;

        init_flag();
        CenterCalMinRow = CenterCalMinRow_tmp;
        CenterCalMaxRow = CenterCalMaxRow_tmp;
        ExSpeed_SUM = ExSpeed_SUM_tmp;
        ips114_clear(BLACK);
        ServoPIDParam[0][0] = kp1_tmp;

    }
}