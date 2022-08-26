//
// Created by Administrator on 2022/3/14.
//

#ifndef MAIN_CPP_TY_H
#define MAIN_CPP_TY_H
#include "zf_common_headfile.h"
 void draw(int x, int y, int flag) ;
void v_to_mat(Mat &image1, unsigned char b[ROW][COL]);
    void mat_to_v(Mat &image1, unsigned char b[ROW][COL]) ;
void ips114_show_gray_image_vec(uint16_t x, uint16_t y, uint8_t *p[][TRFED_COL], uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold) ;
    void ips114_show_string(int a, int b, string c);
    void ips114_draw_point(int x,int y,int a);
    void draw(int x, int y, int flag) ;
        void my_delay(int a);
            void gpio_low(int a);
            void gpio_high(int a);
            void ips114_clear (uint16_t color);
void init_img(string path);

#endif //MAIN_CPP_TY_H
