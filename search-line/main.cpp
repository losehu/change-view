#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include "zf_common_headfile.h"
using namespace cv;
using namespace std;
Mat src;
void mouse_call_back(int event, int mouse_x, int mouse_y, int flags, void *userdata);

int bin_to_dec(int start_num, int start_bit, int cnt) {
    int sum = 0;
    int cnt_bit = 0;
    for (int i = 0; i < cnt; i++) {
        sum += ((img_info[start_num] >> start_bit) & 1) * (pow(2, cnt_bit));
        start_bit++;
        cnt_bit++;
        if (start_bit == 8) {
            start_bit = 0, start_num++;
        }
    }
    return sum;
}
int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c;
}
int main() {
    init_flag();
    fork_flag.cnt = 0;
    init_img("C:/Users/Administrator/Desktop/github/chang-view/search-line/100.png");
    //flip(src, src, 1);
    uint8_t a[ROW][COL];
    for (int i = 5; i < 15; i++) {
        for (int j = COL / 2 - 10; j < COL / 2 + 10; j++) {
            //    src.at<unsigned char>(ROW-1-i,j)=0;
        }
    }
    mat_to_v(src, a);
    uint8_t img[ROW][COL];
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)PerImg_ip[i][j] = &a[i][j];
    Mat show_img;
    src.copyTo(show_img);
    namedWindow("result1", 0);//创建窗口
    resizeWindow("result1", 114 * 5, 100 * 5);
    resize(src, src, Size(114, 100));
    for (int i = 0; i < ROW; i++) for (int j = 0; j < COL; j++) show_img.at<unsigned char>(i, j) = 0;
    cvtColor(src, src, COLOR_GRAY2RGB);
    search_line();
    if (bin_to_dec(0, 0, 8) == 85) cout << "head check ok!" << endl;
    int check_len;
    check_len = bin_to_dec(3, 0, 16);
    if (bin_to_dec((check_len) * 3 / 8 + 5, ((check_len) * 3) % 8, 8) == 189)cout << "check end ok!" << endl;
    int start_num = 0, start_bit = 0, now_setp = 0, check_head = 0;
    int start_x, start_y, step_len, max_y = 0, min_y = 999, min_x = 999, max_x = 0;
    float point_y[check_len + 1], point_x[check_len + 1];
    int point_cnt = 0;
    point_y[0] = start_y, point_x[0] = start_x, point_cnt = 1;
    for (int i = 0; i < 4 + check_len; i++) {
        if (i == 0) {
            check_head = bin_to_dec(start_num, start_bit, 8);
            start_bit += 8;
        } else if (i == 1) {
            start_x = bin_to_dec(start_num, start_bit, 8);
            start_bit += 8;
            cout << "start_x2:" << start_x << endl;
        } else if (i == 2) {
            start_y = bin_to_dec(start_num, start_bit, 8);
            start_bit += 8;
            cout << "start_y2:" << start_y << endl;
            show_img.at<unsigned char>(ROW - 1 - start_y, start_x) = 255;
        } else if (i == 3) {
            step_len = bin_to_dec(start_num, start_bit, 16);
            start_bit += 16;
            cout << "step_leny2:" << step_len << endl;
        } else {
            now_setp = bin_to_dec(start_num, start_bit, 3);
            start_x += img_setp[now_setp][1];
            start_y += img_setp[now_setp][0];
            if (start_y > max_y)max_y = start_y;
            else if (min_y > start_y)min_y = start_y;
            if (start_x > max_x)max_x = start_x;
            else if (min_x > start_x)min_x = start_x;
            show_img.at<unsigned char>(ROW - 1 - start_y, start_x) = 255;
            start_bit += 3;
            point_x[point_cnt] = start_x;
            point_y[point_cnt] = start_y;
            point_cnt++;
        }
        if (start_bit >= 8) {
            start_num += start_bit / 8;
            start_bit = start_bit % 8;
        }
    }
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (j < min_x || j > max_x || i < min_y || i > max_y) {
            } else if (pnpoly(point_cnt, point_x, point_y, (float) j, (float) i))
                show_img.at<unsigned char>( ROW - 1 - i,j) = 255;
        }
    }
    imshow("result1", show_img);
    // setMouseCallback("result", mouse_call_back, 0);
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


