#批量处理图片库，观察准确识别率
#采用神经网络逐个试探参数值，选取识别率最高且耗时最短的参数值
#方法:随机数遍历记录最大识别数和该识别数下最短耗时情况下的取值

'''**************以非调试模式运行以加速程序执行**********'''

# 导入模块
# 导入工具包
import imghdr
from itertools import count
from statistics import mode
import numpy as np
import argparse
from pyzbar import pyzbar
import matplotlib.pyplot as plt
import cv2
import math
from numpy import *;
import threading
import time
import inspect
import ctypes
'''
camera_number = 2
#设置摄像头的分辨率
res_width = 3840   
res_height = 2160
#Find_Max_Matrix函数中二值化区域追踪阈值
binary_edge = 160
#设置程序执行模式 0-表示实时监测 1-表示查看追踪区域效果 2-查看条形码识别效果 3-查看1.2
presentation_mode = 3
dead_time = 5            #设置识别迭代次数
dead_angle = 5           #设置识别终止角度
dead_gray = 10           #图像二值化的临界值
dead_enlarge_heigth = 310#图像高度小于dead_enlarge_heigth的图像将按比例将高放大到enlarge_height
enlarge_height = 700     #低分辨率图像放大到enlarge_height
width_below = 100        #设置条码矩形判断条件，矩形宽大于100
heigth_below = 25        #设置条码矩形判断条件，矩形高大于25
ra_param_below = 1.5     #矩形宽高比下限
ra_param_up = 8          #矩形宽高比上限
rectKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 4))#自定义卷积矩阵
sqKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  #自定义卷积矩阵
'''
#presentation_mode选择展示方式或者实时识别
#twist_angle 和 twist_conbination决定识别模式
#只智能识别:twist_angle=0
#只旋转识别:twist_angle!=0 且twist_conbination=0
#先直接旋转后智能旋转: twist_angle!=0 且twist_conbination=1

start1 = time.time()
res_width = 3840   #设置摄像头的分辨率 宽 矩阵列
res_height = 2160  #设置摄像头的分辨率 高 矩阵行
presentation_mode = 0    #设置程序执行模式 0-表示实时监测 1-表示查看追踪区域效果 2-查看条形码识别效果 3-同时查看1.2
twist_angle = 18         #设置每次直接旋转角度，若值为0则只智能旋转
twist_time = 9          #以twist_angle旋转twist_time
twist_conbination = 1    #twist_conbination=1先直接旋转再智能旋转(具体效果不确定，处理速度稍慢)twist_conbination=0只直接旋转
camera_number = 0        #设置使用的摄像头 0为笔记本摄像头 3为iriun手机摄像头
binary_edge = 155        #Find_Max_Matrix函数中二值化区域追踪阈值
dead_time = 1            #设置识别迭代次数5
dead_angle = 5           #设置识别终止角度5
dead_gray = 20           #图像二值化的临界值20
dead_enlarge_heigth = 310#图像高度小于dead_enlarge_heigth的图像将按比例将高放大到enlarge_height
enlarge_height = 700     #低分辨率图像放大到enlarge_height
width_below = 100        #设置条码矩形判断条件，矩形宽大于100
heigth_below = 25        #设置条码矩形判断条件，矩形高大于25
ra_param_below = 1.5     #矩形宽高比下限1.5
ra_param_up = 8          #矩形宽高比上限8
rectKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 4))#自定义卷积矩阵
sqKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  #自定义卷积矩阵
valid_twist_angle=0
#显示图片，按下任意键退出展示
def cv_show(name,img):
    cv2.imshow(name,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#根据重新设置图像宽高等比缩放图像,常用参数heigth = num，例如resize(image,height=700)
def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))
    resized = cv2.resize(image, dim, interpolation=inter)
    return resized

#得到旋转后的图像
def bar(image, angle,locs):
    if presentation_mode == 2 or presentation_mode==3 or twist_angle!=0 and presentation_mode!=0:
        cv_show("image", resize(image,height=700))
    #进行图像旋转
    bar = rotate_bound(image, angle,locs)
    if presentation_mode == 2 or presentation_mode==3 or twist_angle!=0 and presentation_mode!=0:
        cv_show("bar", resize(bar,height=700))
    #识别旋转后图中的条形码
    return bar

#根据指定参考系按指定角度旋转图像
def rotate_bound(image, angle,locs):
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D((cX, cY), angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    return cv2.warpAffine(image, M, (nW, nH))

def Barcodes_After_bar(image, angle,locs):
    global valid_twist_angle
    if valid_twist_angle!=0:
        return
    # print("thread!!!!!")
    process_image = bar(image, angle,locs)
    if valid_twist_angle!=0:
        return
    barcodes = pyzbar.decode(process_image)
    if valid_twist_angle!=0:
        return
    # cv_show('process_image',process_image)
    if barcodes != []:
        valid_twist_angle=angle
        # print("valid_twist_angle = ",valid_twist_angle)

#递归进行条码识别，通过调整图像角度直至条码水平，返回识别到的条码，否则返回值为空
def Barcode(image):
    global valid_twist_angle
    valid_twist_angle = 0
    #Zbar库自带条形码识别，将识别道德条形码所有信息数据保存在barcodes中
    barcodes = pyzbar.decode(image)
    locs = []
    if barcodes!=[]:
        return barcodes
    #presentation_mode=2 or 3通过单线程逐步旋转进行展示过程
    if presentation_mode == 2 or presentation_mode==3:
        #如果选择twist_angle!=0，直接进入全旋转模式
        if twist_angle!=0:
            angle = twist_angle
            for i in range(1,twist_time+1):
                process_image = bar(image, angle,locs)
                barcodes = pyzbar.decode(process_image)
                angle += twist_angle
                #达到旋转上限或者识别到条码退出循环
                if barcodes!=[]:
                    return barcodes
            if twist_conbination == 0:
                return barcodes
    #多线程并行识别
    if presentation_mode==0:
        angle = twist_angle
        threads = []
        t1 = threading.Thread(target=Barcodes_After_bar, name="Job1", args=(image,1*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t1)
        t2 = threading.Thread(target=Barcodes_After_bar, name="Job2", args=(image,2*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t2)
        t3 = threading.Thread(target=Barcodes_After_bar, name="Job3", args=(image,3*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t3)
        t4 = threading.Thread(target=Barcodes_After_bar, name="Job4", args=(image,4*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t4)
        t5 = threading.Thread(target=Barcodes_After_bar, name="Job5", args=(image,5*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t5)
        t6 = threading.Thread(target=Barcodes_After_bar, name="Job6", args=(image,6*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t6)
        t7 = threading.Thread(target=Barcodes_After_bar, name="Job7", args=(image,7*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t7)
        t8 = threading.Thread(target=Barcodes_After_bar, name="Job8", args=(image,8*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t8)
        t9 = threading.Thread(target=Barcodes_After_bar, name="Job9", args=(image,9*angle,[]))  ## 可见Thread是一个类，需要修改一些默认参数
        threads.append(t9)
        for i in range(0,9):
            threads[i].start()
        #valid_twist_angle!=找到识别成功的角度，退出进程
        while(threading.active_count()>6):
            # print("当前线程的个数:", threading.active_count())
            1
        if valid_twist_angle != 0:
            process_image = bar(image, valid_twist_angle,locs)
            barcodes = pyzbar.decode(process_image)
            print("valid_twist_angle= ",valid_twist_angle)
            return barcodes
    print("start to cut ")
    #第一次读入图片为彩色，旋转之后递归读入均为灰度图
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    #未识别到条码，需要调整条码角度
    if barcodes == []:
        #展示输入原画面
        if presentation_mode == 2 or presentation_mode==3:
            cv_show('image_source',resize(image,height = 700))
        print("image_source_shape",image.shape)
        global count_time
        global dead_time
        global dead_angle
        count_time += 1 #递归次数
        print("count_time = ",end = ""),print(count_time)
        #设置递归次数上限，达到dead_time停止识别
        if count_time > dead_time:
            print("Count_ime is Max")
            if presentation_mode == 2 or presentation_mode==3:
                cv_show("Max Fail image",resize(image,height=700))
            #识别失败，返回空值[]
            return barcodes
        angle,locs = barcode_angle(gray)
        #条码倾斜角度小于dead_angle，该条码不属于可解码类型,否则进行智能旋转再识别
        if angle!=[] and abs(angle)>dead_angle :
            #条码倾斜角度较大，调整至水平
            process_image = bar(gray, angle,locs)
            #递归，对已调整图像再次识别

            valid_area = Find_Max_Matrix(process_image)
            x=valid_area[0]
            y=valid_area[1]
            w=valid_area[2]
            h=valid_area[3]
            if w!=0:
                process_image = process_image[y:(h+y),x:(w+x)]
            else:
                #无物体或未识别到追踪目标
                print("Ok")
                count_time = dead_time
            if presentation_mode!=0:
                cv_show("process_image", resize(process_image,height = 700))

            barcodes = Barcode(process_image)
    #识别成功，返回条码barcodes
    #或未识别到矩形，或条码角度已小于dead_angle但该条码不属于可解码类型
    return barcodes

#获取条形码倾斜角度
def barcode_angle(image):
    if image.shape[0]>700:
        image = resize(image,height=700)

    kernel = np.ones((15, 6), np.uint8)
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    global dead_gray
    gray = cv2.GaussianBlur(gray, (5, 5), 0)                                #高斯滤波降噪
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, rectKernel)           #礼帽提取高亮区域
    if presentation_mode == 2 or presentation_mode==3:
        cv_show("tophat", resize(tophat,height=700))
    gradX = cv2.morphologyEx(tophat, cv2.MORPH_CLOSE, rectKernel)           #闭操作连接条码
    if presentation_mode == 2 or presentation_mode==3:
        cv_show("gradX", resize(gradX,height=700))
    ret, thresh = cv2.threshold(gradX, dead_gray, 255, cv2.THRESH_BINARY)   #二值化
    if presentation_mode == 2 or presentation_mode==3:
        cv_show("thresh", resize(thresh,height=700))
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)             #开操作消除局部影响
    #由二值图像得到图像轮廓
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if presentation_mode == 2 or presentation_mode==3:
        res = cv2.drawContours(image.copy(), contours, -1, (0, 0, 255), 2)
        cv_show("res",resize(res,height=700))
    line_angle = []
    line_locs = []
    line_contours =[]
    line_position = []
    line_angle_sum = 0
    if len(contours) != 0:
        for (i,c) in enumerate(contours):
            #cv2.minAreaRct()返回一个嫩个装下轮廓最小矩形的所有信息
            #左指射线x沿顺时针方向旋转转，最先接触边为w，并返回旋转角度[0,90]
            #存储该矩形信息
            rect = cv2.minAreaRect(contours[i])
            (x1, y1) = cv2.minAreaRect(contours[i])[0]
            (w1,h1) = cv2.minAreaRect(contours[i])[1]
            temp_angle = cv2.minAreaRect(contours[i])[2]
            #转换长边为宽
            if w1<h1:
                temp_angle  -= 90
                s = w1
                w1 = h1
                h1 = s
            #将矩形坐标、宽高转为整型
            m = int(x1-(w1/2)*math.cos(temp_angle))
            q = int(y1+(h1/2)*math.sin(temp_angle))
            x = m
            y = q
            m = int(w1)
            q = int(h1)
            w = m
            h = q
            #计算宽高比
            ar = w / float(h)
            #导入可控参数
            global ra_param_below
            global ra_param_up
            global width_below
            global heigth_below
            #筛选符合条码条件的矩形
            if ar>=ra_param_below and ar<=ra_param_up  and  (w > width_below) and (h >heigth_below):
                temp = cv2.drawContours(image.copy(), [c], 0, (0, 0, 255), 2)
                if presentation_mode == 2 or presentation_mode==3:
                    cv_show("Here Important", resize(temp,height=700))
                line_contours.append(contours[i])
                line_locs.append(rect)
                line_position.append(i)
                line_angle.append(temp_angle)
        #存在条码矩形
        if len(line_position) != 0:
            #计算符合条件矩形倾角平均值
            if presentation_mode == 2 or presentation_mode==3:
                image_rectangle = image.copy()
            for i in range(0,len(line_position)):
                if presentation_mode == 2 or presentation_mode==3:
                    rect = line_locs[i]
                line_angle_sum += line_angle[i]
                if presentation_mode == 2 or presentation_mode==3:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    image_rectangle = cv2.drawContours(image_rectangle, [box], 0, (0, 0, 255), 2)
            if presentation_mode == 2 or presentation_mode==3:
                cv_show("image_rectangle",resize(image_rectangle,height=700))
                res = cv2.drawContours(image.copy(), line_contours, -1, (0, 0, 255), 2)
                cv_show("res_fake", resize(res,height=700))
            line_angle = line_angle_sum/len(line_position)
            print("line_position:"),print(line_position)
            print("line_angle:"),print(line_angle)
            #返回待调整倾角和合格矩阵集
            return line_angle,line_locs
        else:
            #该画面中无条码
            return line_angle,line_locs
    #轮廓值为空，图像画面异常
    else:
        return line_angle,line_locs

def Find_Max_Matrix(image):
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    #cv_show('gray',gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)                                #高斯滤波降噪
    if presentation_mode == 1 or presentation_mode==3:
        gray = cv2.blur(gray, (6, 6))
        cv_show('Blur',resize(gray,height = 700))
    global binary_edge
    ret, thresh = cv2.threshold(gray, binary_edge, 255, cv2.THRESH_BINARY)#二值化
    if presentation_mode == 1 or presentation_mode==3:
        thresh_small = resize(thresh,height = 700)
        cv_show('thresh_small',thresh_small)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)#获取轮廓
    if presentation_mode == 1 or presentation_mode==3:
        temp_image1 = cv2.drawContours(image.copy(),contours,-1, (0, 0, 255), 2)
        cv_show('temp_image1',resize(temp_image1,height = 700))
    dot=[]  # 用来保存所有轮廓返回的坐标点。
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w<500 or w<500:
            continue
        if presentation_mode == 1 or presentation_mode==3:
            temp_image2 = cv2.drawContours(image.copy(),c,-1, (0, 0, 255), 2)
            cv_show('temp_image2',resize(temp_image2,height = 700))
        min_list=[] # 保存单个轮廓的信息，x,y,w,h,area。 x,y 为起始点坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        min_list.append(x)
        min_list.append(y)
        min_list.append(w)
        min_list.append(h)
        min_list.append(w*h) # 把轮廓面积也添加到 dot 中
        dot.append(min_list)
    # 找出最大矩形的 x,y,w,h,area
    valid_area = array([0,0,0,0])
    if dot != []:
        max_area=dot[0][4] # 把第一个矩形面积当作最大矩形面积
        for inlist in dot:
            area=inlist[4]
            if area >= max_area:
                max_area=area
                valid_area = inlist

    return valid_area

def set_res(cap, x,y):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(x))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(y))
    return str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

#主程序
#调用摄像头
vc = cv2.VideoCapture(camera_number)
#设置分辨率
a,b = set_res(vc,res_width,res_height)
print(a,b)
#逐帧读取摄像头画面
if vc.isOpened():
    open, frame = vc.read()
else:
    open = False
while open:
    ret, frame = vc.read()
    if frame is None:
        break
    if ret == True:
        start = time.time()
        count_time=0
        print(frame.shape)
        if presentation_mode == 1 or presentation_mode==3:
            cv_show('frame',resize(frame,height = 700))
        img = frame.copy()
        valid_area = Find_Max_Matrix(img)
        x=valid_area[0]
        y=valid_area[1]
        w=valid_area[2]
        h=valid_area[3]
        # print(x,y,w,h)
        if w!=0:
            frame = frame[y:(h+y),x:(w+x)]
        else:
            print("Ok")
        # cv_show('frame',resize(frame,height =700))
        image = frame
        print(image.shape)
        #显示切割后的画面
        if presentation_mode == 1 or presentation_mode==3:
            cv_show('part_of_image',resize(image,height=700))
        if(image.shape[1] <=dead_enlarge_heigth):
            ratio = image.shape[0] / 900.0
            orig = image.copy()
            image = resize(orig, height =enlarge_height)
        barcodes= Barcode(image)
        if barcodes==[]:
            print("未识别成功")
            end = time.time()
            print("time: ", end =""),print(end-start)
            #存入识别结果
            # cv2.imwrite(image_result_full_F,image)
        else:
            #逐个绘制图片中所有二维码
            for barcode in barcodes:
                #绘制矩形，画在原图上
                (x, y, w, h) = barcode.rect
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 5)
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                text = "{} ({})".format(barcodeData, barcodeType)
                cv2.putText(image, text, (x-20, y), cv2.FONT_HERSHEY_SIMPLEX,.8, (255, 0, 0), 2)
                print("识别成功")
                end = time.time()
                print(barcodeData)
                print("time: ",end =""),print(end-start)
                #存入识别结果
                # cv2.imwrite(image_result_full_S,image)


        end1 = time.time()
        print("Alltime: ", end =""),print(end1-start1)
        image = resize(image,height=300)
        if presentation_mode == 0:
            cv2.imshow("result", image)
        if presentation_mode > 0 :
            cv_show("result", image)
        if (cv2.waitKey(1)>0):#waitKey中参数表示帧数转化等待值
            break
vc.release()
cv2.destroyAllWindows
# 导入工具包
import argparse
from itertools import count
import numpy as np
import argparse
from pyzbar import pyzbar
import matplotlib.pyplot as plt
import cv2
import math
from numpy import *;
import time

# 设置参数
# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required = True,
# 	help = "Path to the image to be scanned")
# args = vars(ap.parse_args())
image = cv2.imread('01.jpg')

def cv_show(name,img):
    cv2.imshow(name,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def order_points(pts):
    # 一共4个坐标点
    rect = np.zeros((4, 2), dtype = "float32")

    # 按顺序找到对应坐标0123分别是 左上，右上，右下，左下
    # 计算左上，右下
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # 计算右上和左下
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect

def four_point_transform(image, pts):
    # 获取输入坐标点
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    # 计算输入的w和h值
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # 变换后对应坐标位置
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # 计算变换矩阵
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    # 返回变换后结果
    return warped

def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))
    resized = cv2.resize(image, dim, interpolation=inter)
    return resized

def Find_Max_Matrix(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)#二值化
    cv_show('thresh',thresh)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)#获取轮廓
    dot=[]  # 用来保存所有轮廓返回的坐标点。
    for c in contours:
        cnt = c#选其中的第一个轮廓
        draw_img = image.copy()
        res = cv2.drawContours(draw_img, [cnt], -1, (0, 0, 255), 2)

        epsilon = 0.1*cv2.arcLength(cnt,True) #True代表闭合曲线
        approx = cv2.approxPolyDP(cnt,epsilon,True)#设置epsilon阈值进行线性近似
        x, y, w, h = cv2.boundingRect(c)
        # 找到边界坐标
        min_list=[] # 保存单个轮廓的信息，x,y,w,h,area。 x,y 为起始点坐标
        x, y, w, h = cv2.boundingRect(c)  # 计算点集最外面的矩形边界
        min_list.append(x)
        min_list.append(y)
        min_list.append(w)
        min_list.append(h)
        min_list.append(w*h) # 把轮廓面积也添加到 dot 中
        dot.append(min_list)
    # 找出最大矩形的 x,y,w,h,area
    max_area=dot[0][4] # 把第一个矩形面积当作最大矩形面积
    valid_area = array([0,0,0,0])
    for inlist in dot:
        area=inlist[4]
        if area >= max_area:
            x=inlist[0]
            y=inlist[1]
            w=inlist[2]
            h=inlist[3]
            max_area=area
            valid_area = inlist
    # 在原图上画出最大的矩形
    draw_img = image.copy()
    res = cv2.rectangle(draw_img, (x, y), (x + w , y + h ), (0, 255, 0), 2)
    print(x,y,w,h)
    cv_show('res',res)
    crop = image[y:(h+y),x:(w+x)]
    cv_show("crop", crop)
    return valid_area

valid_area = Find_Max_Matrix(image)
print(valid_area)


print(approx)
(tl, tr, br, bl) = approx#从右下角顺时针
tl[:,0] =tl[:,0]+5
tl[:,1] =tl[:,1]-5
tr[:,0] =tr[:,0]-5
tr[:,1] =tr[:,1]-5#第一个x第二个y
br[:,0] =br[:,0]-5
br[:,1] =br[:,1]+5#第一个x第二个y
bl[:,0] =bl[:,0]+5
bl[:,1] =bl[:,1]+5#第一个x第二个y
print(tl)
approx[0] = tl
approx[1] = tr
approx[2] = br
approx[3] = bl
print(approx)
res = cv2.drawContours(draw_img, [approx], -1, (0, 0, 255), 2)
cv_show(res,'res')

# 读取输入
# image = cv2.imread(image)
#坐标也会相同变化
ratio = image.shape[0] / 500.0
orig = image.copy()


image = resize(orig, height = 500)

# 预处理
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 0)
edged = cv2.Canny(gray, 75, 200)

# 展示预处理结果
print("STEP 1: 边缘检测")
cv2.imshow("Image", image)
cv2.imshow("Edged", edged)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 轮廓检测
cnts, hierarchy= cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]

# 遍历轮廓
for c in cnts:
    # 计算轮廓近似
    peri = cv2.arcLength(c, True)
    # C表示输入的点集
    # epsilon表示从原始轮廓到近似轮廓的最大距离，它是一个准确度参数
    # True表示封闭的
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)

    # 4个点的时候就拿出来
    if len(approx) == 4:
        screenCnt = approx
        break

# 展示结果
print("STEP 2: 获取轮廓")
cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)
cv2.imshow("Outline", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 透视变换
warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)

# 二值处理
warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
ref = cv2.threshold(warped, 100, 255, cv2.THRESH_BINARY)[1]
cv2.imwrite('scan.jpg', ref)
# 展示结果
print("STEP 3: 变换")
cv2.imshow("Original", resize(orig, height = 650))
cv2.imshow("Scanned", resize(ref, height = 650))
cv2.waitKey(0)