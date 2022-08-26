//
// Created by Administrator on 2022/3/14.
//
#include "ty.h"

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
        src.at<Vec3b>(x, y)[0] = 138;
        src.at<Vec3b>(x, y)[1] = 15;
        src.at<Vec3b>(x, y)[2] = 255;
    }
    else if(flag==6)
    {
        src.at<Vec3b>(x, y)[0] = 200;
        src.at<Vec3b>(x, y)[1] = 15;
        src.at<Vec3b>(x, y)[2] = 150;
    }
    else if(flag==7)
    {
        src.at<Vec3b>(x, y)[0] = 60;
        src.at<Vec3b>(x, y)[1] = 200;
        src.at<Vec3b>(x, y)[2] = 150;
    }
    else if(flag==8)
    {
        src.at<Vec3b>(x, y)[0] = 0;
        src.at<Vec3b>(x, y)[1] = 0;
        src.at<Vec3b>(x, y)[2] = 0;
    }
}
void ips114_draw_point(int x,int y,int a){draw(y,x,a);}
void ips114_show_string(int a, int b, string c){}
void ips114_show_gray_image_vec(uint16_t x, uint16_t y, uint8_t *p[][TRFED_COL], uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold) {}
void ips114_clear (uint16_t color){}
void gpio_high(int a){}
void gpio_low(int a){}
void my_delay(int a){
    imshow("result", src);
    waitKey(1);
}
void init_img(string path)
{

    namedWindow("result", 0);//创建窗口
    resizeWindow("result", 114*5, 100*5);
    src = imread(path, 0);
    //    namedWindow("result", 0);//创建窗口
//    resizeWindow("result", 114*5, 100*5);
    resize(src,src,Size(114,100));



}