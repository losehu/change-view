/**
 //@FileName    :All_Init.c
 //@CreatedDate :2021年12月19日
 //@Author      :LiHao
 //@Description :
**/

#include "All_Init.h"

/**
*@Name			:All_Init
*@Description 	:All_Init
*@Param			:None
*@Return		:None
*@Sample		:All_Init();
**/
void All_Init(void)
{
  //  ImageProcessingStatusInit();
    init_flag();
    GetSimBinImage();
    ImagePerspective_Init();
    Base.element_check = 'T';//元素判断 'T':开 'F':关
}
