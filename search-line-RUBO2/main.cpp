#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include "zf_common_headfile.h"
using namespace cv;
using namespace std;
Mat src;
 uint8_t a[ROW][COL];

void mouse_call_back(int event, int mouse_x, int mouse_y, int flags, void *userdata);
int main() {
    init_flag();
    fork_flag.cnt = 0;
    init_img("C:/Users/Administrator/Desktop/github/chang-view/search-line-RUBO2/9.BMP");
    //flip(src, src, 1);
    imshow("result", src);
    mat_to_v(src, a);

    cvtColor(src, src, COLOR_GRAY2RGB);




    RUBO_IMG_INIT();
     process_image();
  //  waitKey(0);

// 60 50







//        cout << "连通域: " << connect.room << endl;
//    for (int i = ROW - 1; i >= 0; i--) {
//        if ( ImageUsed[i][Img.LeftBorder[ROW - 1 - i]] != 0)
//            draw(i,  Img.LeftBorder[  ROW - 1 -  i],6);
//        if ( ImageUsed[i][Img.RightBorder[ROW - 1 - i]] != 0)
//            draw(i,Img.RightBorder[ ROW -1 -i],    4);
//       draw(i, Img.Center[ROW - 1 - i], 5);
//        waitKey(1);
//        imshow("result", src);
//    }
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


