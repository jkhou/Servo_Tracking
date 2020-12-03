# -*- coding:utf-8 -*-

import cv2
import time
import datetime
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
from color_feature import color_block_finder,draw_color_block_rect



btm_kp = 5 # 底部舵机的Kp系数
top_kp = 5 # 顶部舵机的Kp系数
btm_ki = 0 # 底部舵机的Ki系数
top_ki = 0 # 顶部舵机的Ki系数
btm_kd = 0 # 底部舵机的Kd系数
top_kd = 0 # 顶部舵机的Kd系数

btm_cp=0
btm_ci=0
btm_cd=0
top_cp=0
top_ci=0
top_cd=0

global last_btm_degree # 上一次底部舵机的角度
global last_top_degree # 上一次顶部舵机的角度

global face_x  #追踪点的x坐标
global face_y  #追踪点的y坐标
global img

last_btm_degree = 270   # 下舵机初值设定为270度
last_top_degree = 90    # 上舵机初值设定为90度

offset_dead_block = 0.1 # 设置偏移量的死区

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
    global prevtime  #上一时刻的时间,从主函数读出  
    global btm_ci
    global last_btm_degree # 上一次底部舵机的角度

    currtime=time.time()
    dt=currtime-prevtime  #时间的微小增量
    
    # 设置最小阈值
    if abs(offset_x) < offset_dead_block:
       offset_x = 0

    btm_cp=offset_x #比例控制的乘数
    btm_ci+=offset_x*dt #积分控制的乘数
    btm_cd=offset_x/dt #微分控制的乘数

    # offset范围在-50到50左右
    # delta_degree = btm_cp*btm_kp+btm_ci*btm_ki+btm_cd*btm_kd
    delta_degree = btm_cp*btm_kp

    print("delta degree of btm:")
    print(delta_degree)

    # 计算得到新的底部舵机角度
    next_btm_degree = last_btm_degree + delta_degree

    print("next_btm_degree:")
    print(next_btm_degree)

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
    global prevtime  #上一时刻的时间,从主函数读出   
    global top_ci
    global last_top_degree # 上一次顶部舵机的角度

    currtime=time.time()
    dt=currtime-prevtime  #时间的微小增量

    # 如果偏移量小于阈值就不相应
    if abs(offset_y) < offset_dead_block:
        offset_y = 0
    
    top_cp=offset_y #比例控制的乘数
    top_ci+=offset_y*dt #积分控制的乘数
    top_cd=offset_y/dt #微分控制的乘数

    # offset范围在-50到50左右
    #delta_degree = top_cp*top_kp+top_ci*top_ki+top_cd*top_kd
    delta_degree = top_cp*top_kp

    # 新的顶部舵机角度
    next_top_degree = last_top_degree - delta_degree
    # 添加边界检测,防止舵机堵转
    if next_top_degree < 0:
        next_top_degree = 90
    elif next_top_degree > 360:
        next_top_degree = 90

    return int(next_top_degree)



def calculate_offset(img_width, img_height, face_x, face_y):

    '''
    计算色块在画面中的偏移量
    偏移量的取值范围： [-1, 1]
    '''
  
    offset_x = float(face_x*1.0 / img_width*1.0 - 0.5) * 2
    print("offset_x:")
    print(offset_x)

    # 在画面中心Y轴上的偏移量
    offset_y = float(face_y*1.0 / img_height*1.0 - 0.5) * 2
    print("offset_y:")
    print(offset_y)

    return (offset_x, offset_y)



class image_converter:

  def __init__(self):
    #发布给ros控制舵机的发布器
    self.servo_pub = rospy.Publisher("/current_servo/pose_setpoint",Vector3, queue_size=1)

    self.bridge = CvBridge()
    #图像订阅
    self.image_sub = rospy.Subscriber("/MVcam/image",Image,self.callback)

    #追踪点坐标订阅
    self.drone_center = rospy.Subscriber("/drone/center",Vector3,self.dronecenter_callback)


  def dronecenter_callback(center):
    global face_x  #飞机的中心像素x位置
    global face_y  #飞机的中心像素y位置

    face_x = center.x
    face_y = center.y


  def callback(self,data):
    global last_btm_degree # 上一次底部舵机的角度
    global last_top_degree # 上一次顶部舵机的角度
    global img

    cv2.namedWindow('Image window', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)

    try:
      img = self.bridge.imgmsg_to_cv2(data, "mono8")

      if img is not None:

          img_height, img_width = img.shape
          print("img h:{} w:{}".format(img_height, img_width))

          ret, img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
          _, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
          max_area = 0
          max_area_index = -1
          print(len(contours))
          for index in range(len(contours)):
              area = cv2.contourArea(contours[index])
              if(area > max_area):
                  max_area = area
                  max_area_index = index

          cx = img_width/2   #追踪点默认设定为画面中心点
          cy = img_height/2
          if max_area_index != -1:
              M = cv2.moments(contours[max_area_index])
              cx = int(M['m10']/M['m00'])
              cy = int(M['m01']/M['m00'])

          print((cx, cy))
          cv2.circle(img, (cx, cy), 2, 255, 0)

          cv2.imshow("Image window", img)
          cv2.waitKey(5)

          global prevtime
          prevtime = time.time()

          # 计算x轴与y轴的偏移量
          (offset_x, offset_y) = calculate_offset(img_width, img_height, cx, cy)

          # 计算下一步舵机要转的角度
          next_btm_degree = btm_servo_control(offset_x)
          next_top_degree = top_servo_control(offset_y)

          print("X轴偏移量：{} Y轴偏移量：{}".format(offset_x, offset_y))
          print('底部角度： {} 顶部角度：{}'.format(next_btm_degree, next_top_degree))

          # 设置舵机旋转角度,发布Vector3(x,y,z)
          self.servo_pub.publish(Vector3(next_btm_degree,next_top_degree,0))

          # 更新角度值
          last_btm_degree = next_btm_degree
          last_top_degree = next_top_degree
      
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
  cv2.destroyAllWindows()


if __name__ == '__main__':
  
  # if  cv2.waitKey(10) & 0xFF == 'q':
  #   cv2.destroyAllWindows()

  # # # 创建底部舵机Kp的滑动条
  # cv2.createTrackbar('BtmServoKp*10','Image window',0, 100,update_btm_kp)
  # # 设置btm_kp的默认值
  # cv2.setTrackbarPos('BtmServoKp*10', 'Image window', btm_kp)
  # # 创建顶部舵机Kp的滑动条
  # cv2.createTrackbar('TopServoKp*10','Image window',0, 100,update_top_kp)
  # # 设置top_kp的默认值
  # cv2.setTrackbarPos('TopServoKp*10', 'Image window', top_kp)


  # # 创建底部舵机Ki的滑动条
  # cv2.createTrackbar('BtmServoKi*10','Image window',0, 70,update_btm_ki)
  # # 设置btm_ki的默认值
  # cv2.setTrackbarPos('BtmServoKi*10', 'Image window', btm_ki)
  # # 创建顶部舵机Ki的滑动条
  # cv2.createTrackbar('TopServoKi*10','Image window',0, 70,update_top_ki)
  # # 设置top_ki的默认值
  # cv2.setTrackbarPos('TopServoKi*10', 'Image window', top_ki)


  # # 创建底部舵机Kd的滑动条
  # cv2.createTrackbar('BtmServoKd*100000','Image window',0, 100,update_btm_kd)
  # # 设置btm_kd的默认值
  # cv2.setTrackbarPos('BtmServoKd*100000', 'Image window', btm_kd)
  # # 创建顶部舵机Kd的滑动条
  # cv2.createTrackbar('TopServoKd*100000','Image window',0, 100,update_top_kd)
  # # 设置top_kd的默认值
  # cv2.setTrackbarPos('TopServoKd*100000', 'Image window', top_kd)

  main()
