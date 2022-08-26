#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include "zf_common_headfile.h"
using namespace cv;
using namespace std;
Mat src;
void mouse_call_back(int event, int mouse_x, int mouse_y, int flags, void *userdata);
int num_right[]={0,1,2,3,4,5,6,7,8,9,10,11,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79};
int num_left[]={13,14,15,16,17,18,19,20,21,22,23,24,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112};
int cnr=18;
string path;
int main(int argc, char *argv[]) {
if(circle_left)     path="C:\\Users\\Administrator\\Desktop\\github\\环岛图片\\circle\\data";//left
else      path="C:\\Users\\Administrator\\Desktop\\github\\环岛图片\\enter_circle_right\\data";//5.Tux.bmp

    string   path_tmp;


    while(1) {
        init_flag();
        fork_flag.cnt = 0;
        if(circle_left)      cnt=cnr;
        else    cnt=num_right[cnr];
        path_tmp=path+to_string(cnt)+".Tux.bmp";
        cout<<cnt<<"\t"<<cnr<<endl;
        init_img(path_tmp);
       //  init_img("C:\\Users\\Administrator\\Desktop\\github\\chang-view\\search-line-RUBO\\2.bmp");
//11 22 24 25 26 40 41 42 43 44 55 56 57 58 59 61 62 63 76 77 80 81 82 83 84 85 86 87 89 90 91 92 93 94 95 96 101 102 103 104
     //111 112
     // flip(src, src, 1);

        uint8_t a[ROW][COL];
//    imshow("result", src);
//    for(int i=0;i<ROW;i++)
//        for(int j=COL/2;j<COL;j++)
//            src.at<unsigned char>(ROW-1-i,j)=255;



        imshow("result", src);
        mat_to_v(src, a);
        uint8_t img[ROW][COL];
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)PerImg_ip[i][j] = &a[i][j];
 cvtColor(src, src, COLOR_GRAY2RGB);
       search_line_fork();
//       GetBorder();
      //  search_line();
     //search_line();
 //   solve_line();
        // cal_middle_line();
      //  cout << "连通域: " << connect.room << endl;
//        for (int i = 0; i < ROW; i++) {
//            if (Img.LeftBorderFindFlag[i] == 'T' || Img.RightBorderFindFlag[i] == 'T' ) {
//            draw(ROW - 1 - i, Img.LeftBorder[i], 4);
//            draw(ROW - 1 - i, Img.RightBorder[i], 2);
////            draw(ROW - 1 - i, Img.Center[i], 5);
//            }
//        }


        imshow("result", src);
        waitKey(1);
        setMouseCallback("result", mouse_call_back, 0);
        waitKey(0);
        cnr++;
    }
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


