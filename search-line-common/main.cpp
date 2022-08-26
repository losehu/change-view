#include <bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include"ty.h"
using namespace cv;
using namespace std;
int main() {

    namedWindow("result", 0);//´´½¨´°¿Ú
    resizeWindow("result", 114*8, 100*5);
    src = imread("../9.bmp", 0);
    mat_to_v(src,ImageUsed);
    Mat drc=src;
      cvtColor(src, src, COLOR_GRAY2RGB);
    search_line();
     solve_line() ;
         cal_middle_line();

        for(int i=0;i<ROW;i++) {
        draw(i, Img.LeftBorder[ROW-1-i], 3);
        draw(i, Img.RightBorder[ROW-1-i], 4);
            draw(i,  Img.Center[ROW-1-i], 5);


        }
    imshow("result", src);
    waitKey(0);
}


