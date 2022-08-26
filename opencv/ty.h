//
// Created by rubo on 2022/1/2.
//

#ifndef MAIN_CPP_TY_H
#define MAIN_CPP_TY_H
#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;
#define num_type unsigned char
 int width =188;
int high =120;
 int width_end =188;
int high_end =120;
/*
void v_to_mat1(Mat &image1, num_type b[][width_end]) { // 数组转矩阵
    for (int i = 0; i < high_end; i++)
        for (int j = 0; j < width_end; j++)
            image1.at<uchar>(i, j) = b[i][j];
}*/
/*
void mat_to_v_double(Mat &image1, double b[][3]) {// 矩阵转数组
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            b[i][j] = image1.at<double>(i, j);
}*/
/*void mat_to_v(Mat &image1, num_type b[][width]) {// 矩阵转数组

}
*/


#endif //MAIN_CPP_TY_H
