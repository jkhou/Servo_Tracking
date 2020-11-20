# -*- coding:utf-8 -*-
'''
人脸识别并控制舵机进行人脸追踪

NOTE
1. 为保证人脸识别的帧率分辨率控制在320x240
BUG
1. 刚开始的时候缓存区视频流的问题， 导致舵机云台会乱转 向大佬低头  数值越界
ESP32出解析的数据：Bottom: 6553600 Top32有时候不执行
TODO
1. 获得舵机旋转角度的反馈数据
2. 创建两个Trackbar， 设置两个Kp取值
3. 绘制上下臂舵机的波形图

参考Kp取值
1. Kp = 10  比例系数较大，来回抖动非常明显
2. Kp = 20  幅度过大，旋转后目标直接丢失: 6556160
2. Reset 舵机云台，串口数据发送之后，ESP
3. Kp = 5   幅度适中，有小幅抖动
4. Kp = 2   相应速度比较慢
'''


import cv2
import time
import datetime
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
from color_feature import color_block_finder,draw_color_block_rect


# def __init__(self):
#     self.servo_pub = rospy.Publisher("servo/pose",Vector3)


last_btm_degree = 270 # 最近一次底部舵机的角度值记录
last_top_degree = 270 # 最近一次顶部舵机的角度值记录

btm_kp = 5 # 底部舵机的Kp系数
top_kp = 5 # 顶部舵机的Kp系数
btm_ki = 1 # 底部舵机的Ki系数
top_ki = 1 # 顶部舵机的Ki系数
btm_kd = 1 # 底部舵机的Kd系数
top_kd = 1 # 顶部舵机的Kd系数

btm_cp=0
btm_ci=0
btm_cd=0
top_cp=0
top_ci=0
top_cd=0

offset_dead_block = 0.25 # 设置偏移量的死区

# 舵机角度初始化
#set_cloud_platform_degree(last_btm_degree, last_top_degree)
#set_cloud_platform_degree(last_btm_degree, last_top_degree)

# 创建一个窗口 名字叫做Face
#cv2.namedWindow('ColorDetect',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

def update_btm_kp(value):
    # 更新底部舵机的比例系数
    global btm_kp
    btm_kp = value*0.1

def update_top_kp(value):
    # 更新顶部的比例系数
    global top_kp
    top_kp = value*0.1

def update_btm_ki(value):
    # 更新底部舵机的积分系数
    global btm_ki
    btm_ki = value*0.1

def update_top_ki(value):
    # 更新顶部的积分系数
    global top_ki
    top_ki = value*0.1

def update_btm_kd(value):
    # 更新底部舵机的微分系数
    global btm_kd
    btm_kd = value*0.00001

def update_top_kd(value):
    # 更新顶部的微分系数
    global top_kd
    top_kd = value*0.00001



def btm_servo_control(offset_x):
    '''
    底部舵机的比例控制
    这里舵机使用开环控制
    '''
    global offset_dead_block # 偏移量死区大小
    global btm_kp # 控制舵机旋转的比例系数
    global btm_ki # 控制舵机旋转的积分系数
    global btm_kd # 控制舵机旋转的微分系数
    global last_btm_degree # 上一次底部舵机的角度
    global prevtime  #上一时刻的时间,从主函数读出  
    global btm_ci 

    print(last_btm_degree)
    ##print('@@@@@@@@@@@@')

    currtime=time.time()
    dt=currtime-prevtime  #时间的微小增量
    
    # 设置最小阈值
    if abs(offset_x) < offset_dead_block:
       offset_x = 0

    btm_cp=offset_x #比例控制的乘数
    btm_ci+=offset_x*dt #积分控制的乘数
    btm_cd=offset_x/dt #微分控制的乘数

    # offset范围在-50到50左右
    delta_degree = btm_cp * btm_kp+btm_ci*btm_ki+btm_cd*btm_kd

    print(delta_degree)
    ##print('%$$$$$$$$$$$$$$$$$$$$$4')

    # 计算得到新的底部舵机角度
    next_btm_degree = last_btm_degree + delta_degree
    print(next_btm_degree)
    ##print('!!!!!!!!!')
    # 添加边界检测,防止舵机堵转
    if next_btm_degree < 0:
        next_btm_degree = 270
    elif next_btm_degree > 360:
        next_btm_degree = 270
    
    return int(next_btm_degree)


def top_servo_control(offset_y):
    '''
    顶部舵机的比例控制
    这里舵机使用开环控制
    '''
    global offset_dead_block
    global top_kp # 控制舵机旋转的比例系数
    global top_ki # 控制舵机旋转的积分系数
    global top_kd # 控制舵机旋转的微分系数
    global last_top_degree # 上一次顶部舵机的角度
    global prevtime  #上一时刻的时间,从主函数读出   
    global top_ci

    currtime=time.time()
    dt=currtime-prevtime  #时间的微小增量

    # 如果偏移量小于阈值就不相应
    if abs(offset_y) < offset_dead_block:
        offset_y = 0
    
    top_cp=offset_y #比例控制的乘数
    top_ci+=offset_y*dt #积分控制的乘数
    top_cd=offset_y/dt #微分控制的乘数

    # offset范围在-50到50左右
    delta_degree = top_cp*top_kp+top_ci*top_ki+top_cd*top_kd
    # 新的顶部舵机角度
    next_top_degree = last_top_degree - delta_degree
    # 添加边界检测,防止舵机堵转
    if next_top_degree < 0:
        next_top_degree = 270
    elif next_top_degree > 360:
        next_top_degree = 270
    
    return int(next_top_degree)


def rect_filter(rects):
    '''
    对色块进行一个过滤
    '''
    if len(rects) == 0:
        return None
    
    # 目前找的是画面中面积最大的色块
    max_rect =  max(rects, key=lambda rect: rect[2]*rect[3])
    (x, y, w, h) = max_rect
    if w < 10 or h < 10:#单位是像素
        return None
    return max_rect


def calculate_offset(img_width, img_height, face):
    '''
    计算色块在画面中的偏移量
    偏移量的取值范围： [-1, 1]
    '''
    (x, y, w, h) = face
    face_x = float(x + w/2.0)
    face_y = float(y + h/2.0)
    # 在画面中心X轴上的偏移量
    offset_x = float(face_x / img_width - 0.5) * 2
    # 在画面中心Y轴上的偏移量
    offset_y = float(face_y / img_height - 0.5) * 2

    return (offset_x, offset_y)



class image_converter:

  def __init__(self):
    self.servo_pub = rospy.Publisher("/servo/pose",Vector3)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/collect_image/image",Image,self.callback)


  def callback(self,data):
    try:
        #print('Start!')
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        if img is not None:
            img_height, img_width,_ = img.shape
            #print("img h:{} w:{}".format(img_height, img_width))

            cv2.imshow("Image window", img)
            cv2.waitKey(3)
    
        # 颜色阈值下界(HSV) lower boudnary
        lowerb = (149, 147, 142)
        # 颜色阈值上界(HSV) upper boundary
        upperb = (202, 212, 220)

        #检测画面中的红色色块，识别色块并获得矩形区域数组
        rects = color_block_finder(img, lowerb, upperb, min_w=10, min_h=10)
        # 绘制色块的矩形区域
        canvas = draw_color_block_rect(img, rects)
        # 在HighGUI窗口 展示最终结果 更新画面
        cv2.imshow('ColorDetect', canvas)


        # 做适当的延迟，每帧延时0.1s钟
        #if cv2.waitKey(50) & 0xFF == 'q':
            #break

        # 色块过滤
        rect = rect_filter(rects)

        global prevtime
        prevtime=time.time()

        if rect is not None:
            # 当前画面有色块
            (x, y, w, h) = rect
            print('OK!')

            dt_ms = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f') # 含微秒的日期时间，来源 比特量化
            print(dt_ms)
            #cv2.imwrite('%s.jpg'%('pic_'+str(dt_ms)),img)  #写出视频图片.jpg格式
            # 在原彩图上绘制矩形
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 3)

            # img_height, img_width,_ = img.shape
            # print("img h:{} w:{}".format(img_height, img_width))
            

            # 计算x轴与y轴的偏移量
            (offset_x, offset_y) = calculate_offset(img_width, img_height, rect)

            # 计算下一步舵机要转的角度
            next_btm_degree = btm_servo_control(offset_x)
            next_top_degree = top_servo_control(offset_y)
            print("X轴偏移量：{} Y轴偏移量：{}".format(offset_x, offset_y))
            print('底部角度： {} 顶部角度：{}'.format(next_btm_degree, next_top_degree))

            degree = Vector3()

            # 设置舵机旋转角度,发布Vector3(x,y,z)
            self.servo_pub.publish(Vector3(next_btm_degree,next_top_degree,0))
            
            #set_cloud_platform_degree(next_btm_degree, next_top_degree)
        
            # 更新角度值
            last_btm_degree = next_btm_degree
            last_top_degree = next_top_degree
        

            # print("X轴偏移量：{} Y轴偏移量：{}".format(offset_x, offset_y))
            # print('底部角度： {} 顶部角度：{}'.format(next_btm_degree, next_top_degree))


      
    except CvBridgeError as e:
        print(e)


def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':

    cv2.namedWindow('ColorDetect', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)

    # 创建底部舵机Kp的滑动条
    cv2.createTrackbar('BtmServoKp*10','ColorDetect',0, 200,update_btm_kp)
    # 设置btm_kp的默认值
    cv2.setTrackbarPos('BtmServoKp*10', 'ColorDetect', btm_kp)
    # 创建顶部舵机Kp的滑动条
    cv2.createTrackbar('TopServoKp*10','ColorDetect',0, 200,update_top_kp)
    # 设置top_kp的默认值
    cv2.setTrackbarPos('TopServoKp*10', 'ColorDetect', top_kp)


    # 创建底部舵机Ki的滑动条
    cv2.createTrackbar('BtmServoKi*10','ColorDetect',0, 70,update_btm_ki)
    # 设置btm_ki的默认值
    cv2.setTrackbarPos('BtmServoKi*10', 'ColorDetect', btm_ki)
    # 创建顶部舵机Ki的滑动条
    cv2.createTrackbar('TopServoKi*10','ColorDetect',0, 70,update_top_ki)
    # 设置top_ki的默认值
    cv2.setTrackbarPos('TopServoKi*10', 'ColorDetect', top_ki)


    # 创建底部舵机Kd的滑动条
    cv2.createTrackbar('BtmServoKd*100000','ColorDetect',0, 100,update_btm_kd)
    # 设置btm_kd的默认值
    cv2.setTrackbarPos('BtmServoKd*100000', 'ColorDetect', btm_kd)
    # 创建顶部舵机Kd的滑动条
    cv2.createTrackbar('TopServoKd*100000','ColorDetect',0, 100,update_top_kd)
    # 设置top_kd的默认值
    cv2.setTrackbarPos('TopServoKd*100000', 'ColorDetect', top_kd)

    main()

