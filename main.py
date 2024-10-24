#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from math import pi
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan

class main_jeju:
    def __init__(self):
        ## 구조설명
        # 아래에 선언된 publiser main_cmd_~~ 토픽을 각 주행 코드(lane,waypoint,회피주행 코드)에서 구독함
        # 만약 control_drive의 특정 조건이 만족되면 main_cmd_~~의 토픽을 각각 True 또는 False를 발행함
        # main_cmd_~~의 토픽을 주행코드 들이 구독하여 main_cmd_~~가 True일때만 cmd_vel을 발행하도록 코드 수정필요할것
        
        # 노드 초기화 추가
        rospy.init_node('main_jeju', anonymous=True)  
        # Joy 변수 선언 및 초기화
        self.auto = 0
        self.manual = 0
        # utm 변수 선언 및 초기화
        self.my_pose_utm_x = 0
        self.my_pose_utm_y = 0
        
        ## 위치 기반 트랙 구분 리스트
        #############################################필독####################################################################################3
        #역주행할때 리스트를 따로 만드는 이유는 도착점 반경 1m안을 도착점으로 판단하는데 도착점 선에 딱 맞춰서 점을 찍으면 도착점 선 넘기전에 차가 멈추기 때문에 따로 만듦
        # 시작 지점, 라인인식<>웨이포인트 바뀌는 지점, 종료 지점 좌표
        self.flag_point_list = [(260057.470122,3681137.99802), (260071.524,3681151.268), 
                                (260078.255,3681157.550), (260070.127,3681161.09), 
                                (260054.508,3681166.924), (260046.039,3681179.130), 
                                (260020.600,3681148.684), (260032.05,3681155.857), 
                                (260051.892,3681164.780), (260051.98376,3681143.92638)] # 시작점>도착점순으로 주행할때 쓰는 점 리스트  도착점 좌표가 도착점보다 멂
        
        # self.flag_point_list = [(260051.98376,3681143.92638), (260051.892,3681164.780),
        #                         (260032.05,3681155.857), (260020.600,3681148.684),
        #                         (260046.039,3681179.130), (260054.508,3681166.924),
        #                         (260070.127,3681161.09), (260078.255,3681157.550),
        #                         (260071.524,3681151.268), (260057.470122,3681137.99802)] # 도착점>출발점으로 주행할때 쓰는 점 리스트 시작점 좌표가 시작점보다 멂
        
        ## 주행 모드 플래그 초기화
        self.mode_flag = 0
        # 0은 출발 전 
        # 1은 라인인식 주행
        # 2는 웨이포인트 주행
        # 3는 주행 종료
        
        
        ## 장애물 플래그 초기화
        self.obs_flag = 0
        # 0은 장애물 판단이 되지 않았을때
        # 1은 장애물 판단이 되었을때
        

        # ## 장애물 감지 변수 초기화
        # self.is_obstacle = False
        
        self.drive = Twist()
        
        #Lidar 변수 선언
        self.obstacle_ranges = []  # Initializing obstacle_ranges list with empty value
        self.lidar_flag = False
        self.ranges_length = None  # Initializing ranges_length variable with None value
        self.scan_degree = 10  # Initializing scan_degree variable with 60 value

        # Subscriber 선언
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber('/utm_x_topic', Float64, self.utm_x_callback)
        rospy.Subscriber('/utm_y_topic', Float64, self.utm_y_callback)
        # rospy.Subscriber('/usb_cam/image_raw',Image, self.image_callback)

        # Publisher 선언(장애물 회피 노드, lane 주행 노드, waypoint 주행 노드)
        self.pub_obs_toggle = rospy.Publisher('main_cmd_obs', Bool, queue_size=10) # 회피주행이 cmd_vel을 발행하게 하는 토픽 선언
        self.pub_way_toggle = rospy.Publisher('main_cmd_way', Bool, queue_size=10) # 웨이포인트주행이 cmd_vel을 발행하게 하는 토픽 선언
        self.pub_lane_toggle = rospy.Publisher("main_cmd_lane", Bool, queue_size=10) # 라인인식주행이 cmd_vel을 발행하게 하는 토픽 선언
        self.pub_drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # Joy를 통해 manual주행 또는 정지할때 cmd_vel을 발행하게 하는 토픽 선언

        rospy.Timer(rospy.Duration(0.05), self.control_drive)
        
    def update_flags(self): 
        # lane 주행과 waypoint 주행, 장애물 주행을 구분하기 위한 함수
        # 위치 기반으로 lane 주행과 waypoint 주행 구분
        # 각 구간의 시작위치를 지나갈 때 주행 모드(주행 플래그)를 업데이트
        # 각 구간의 시작위치의 반경 r 내에 들어가면 업데이트
        # 장애물 인식은 라바콘 색상 + 라이다 거리로 판단
        # lane 주행 중 차선이 2개 검출되지 않으면 waypoint 주행으로 전환하면 좋을 것 같음
    
        judgment_radius = 1 # 허용반경(m)

        if self.mode_flag == 0 and self.distance_from_flag_point(0) < judgment_radius**2:
            print("start point, lane detection start1")
            self.mode_flag = 1
        elif self.distance_from_flag_point(1) < judgment_radius**2:
            print("lane detection end, waypoint start2")
            self.mode_flag = 2
        elif self.distance_from_flag_point(2) < judgment_radius**2:
            print("waypoint end, lane detection start3")
            self.mode_flag = 1
        elif self.distance_from_flag_point(3) < judgment_radius**2:
            print("lane detection end, waypoint start4")
            self.mode_flag = 2
        elif self.distance_from_flag_point(4) < judgment_radius**2:
            print("waypoint end, lane detection start5")
            self.mode_flag = 1
        elif self.distance_from_flag_point(5) < judgment_radius**2:
            print("lane detection end, waypoint start6")
            self.mode_flag = 2
        elif self.distance_from_flag_point(6) < judgment_radius**2:
            print("waypoint end, lane detection start7")
            self.mode_flag = 1
        elif self.distance_from_flag_point(7) < judgment_radius**2:
            print("lane detection end, waypoint start8")
            self.mode_flag = 2
        elif self.distance_from_flag_point(8) < judgment_radius**2:
            print("waypoint end, lane detection start9")
            self.mode_flag = 1
        elif self.distance_from_flag_point(9) < judgment_radius**2:
            print("end point, stop10")
            self.mode_flag = 3
        
        # 장애물 감지에 따른 obs_flag 업데이트
        if self.obstacle_exit:
            self.obs_flag = 1
        else:
            self.obs_flag = 0

    def distance_from_flag_point(self, num):
        # 현재 위치와 각 구간의 point 사이의 거리 제곱 계산
        return (self.flag_point_list[num][0] - self.my_pose_utm_x)**2 + (self.flag_point_list[num][1] - self.my_pose_utm_y)**2
    
    def utm_x_callback(self, data):
        self.my_pose_utm_x = data.data

    def utm_y_callback(self, data):
        self.my_pose_utm_y = data.data
    
    def joy_callback(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
    
    def laser_scan_callback(self, msg): # LiDAR_scan에서 계산한 값을 저장
        OBSTACLE_PERCEPTION_BOUNDARY = (20)  # Initializing OBSTACLE_PERCEPTION_BOUNDARY variable with 20 value
        self.msg = (msg)  # Storing the message received from "/scan" topic in self.msg variable
        if (len(self.obstacle_ranges) > OBSTACLE_PERCEPTION_BOUNDARY):  # Checking if the length of obstacle_ranges list is greater than OBSTACLE_PERCEPTION_BOUNDARY value
            self.obstacle_exit = True  # Setting obstacle_exit variable to True 장애물을 발견했다는 판단기준
        else:
            self.obstacle_exit = False  # Setting obstacle_exit variable to False
        #print(self.obstacle_ranges)  # Emptying the obstacle_ranges list
    
 
       
    def LiDAR_scan(self): # /scan토픽에서 받아온 값을 계산
        # Create an empty list to store detected obstacles
        obstacle = []
        # If the LiDAR has not been initialized, initialize it
        if self.lidar_flag == False: ###########뭔가 이상함.... 아래에서 true로 바꾸고 초기화 하는게 없음
            # Convert the angle increment values to degrees for the given number of ranges
            
            self.degrees = [
                (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                for i, data in enumerate(self.msg.ranges)
            ]
            # Record the length of the ranges
            self.ranges_length = len(self.msg.ranges)
            # Set the flag to True to indicate that the LiDAR is initialized
            self.lidar_flag = True
            
        # Loop over the ranges in the message and check for obstacles
        for i, data in enumerate(self.msg.ranges):
            # If a distance measurement is less than 0.3m and the angle is within the scan degree range,
            # append the index of that measurement to the obstacle list and record the distance
            if 0 < data < 0.5 and -self.scan_degree < self.degrees[i] < self.scan_degree: # !!!!!!!!!!!!!!!!!!!중요!!!!!!!!!!!!!!!!!!!!!!!! 0.3은 0.3m 수정 필요
                obstacle.append(i)
                self.dist_data = data
                # rospy.logdebug("Obstacle detected at index {}, distance: {}".format(i, data))

        # If any obstacles were detected, calculate their start and end indices and extract the range data
        if obstacle:
            first = obstacle[0]
            first_dst = first
            last = obstacle[-1]
            last_dst = self.ranges_length - last
            self.obstacle_ranges = self.msg.ranges[first : last + 1]
            # If no obstacles were detected, set the indices and distance values to 0
            # rospy.loginfo("Obstacles detected from index {} to {}.".format(first, last))
        
        else:
            first, first_dst, last, last_dst = 0, 0, 0, 0
            rospy.loginfo("No obstacles detected.")

        # Return the start and end indices of the detected obstacles and the indices of the first and last ranges
        # that did not detect an obstacle
        return first, first_dst, last, last_dst
          
    def control_drive(self, event):
        ## 타이머 함수
        # 주행 모드 플래그를 50ms 마다 업데이트
        # 조이스틱 모드/자율주행 모드 선택
        # 각 모드에 따른 주행 코드 실행
        self.update_flags()
        
        if self.manual == 1:
            self.manual_drive()
        elif self.auto == 1:
            self.auto_drive()
        else:
            self.stop_drive()

    def manual_drive(self):
        # manual 모드에서의 주행 코드
        self.speed = self.joy_axes[4]
        self.steering = self.joy_axes[0]
        self.drive.linear.x = self.speed * 0.2 # !!!!!!!!!!!!!!!!!!!중요!!!!!!!!!!!!!!!!!!!!!!!! 속도 수정 필요
        self.drive.angular.z = int(self.steering * 40) # !!!!!!!!!!!!!!!!!!!중요!!!!!!!!!!!!!!!!!!!!!!!! 조향각 수정 필요
        self.pub_drive.publish(self.drive)
        self.pub_obs_toggle.publish(False)
        self.pub_way_toggle.publish(False)
        self.pub_lane_toggle.publish(False)
    
    def auto_drive(self):
        # auto 모드에서의 주행 코드
        if self.obs_flag == 1:  # 장애물 판별 기준 예시
            print("장애물 감지!!")
            self.pub_obs_toggle.publish(True)
            self.pub_way_toggle.publish(False)
            self.pub_lane_toggle.publish(False)  # 3초 이후에는 라인 인식 모드또는 웨이포인트 모드로 전환
            # self.obstacle_detected_time = None  # 타이머 리셋                
        elif self.mode_flag == 1:   # 라인인식 주행
            self.pub_obs_toggle.publish(False)
            self.pub_way_toggle.publish(False)
            self.pub_lane_toggle.publish(True)
            ## 추가 안전 장치(차선이 2개 검출되지 않으면 웨이포인트 주행으로 전환)   
            # if 라인이 2개 검출되지 않으면:
            # self.pub_obs_toggle.publish(False)
            # self.pub_way_toggle.publish(True)
            # self.pub_lane_toggle.publish(False)
        elif self.mode_flag == 2:   # 웨이포인트 주행
            self.pub_obs_toggle.publish(False)
            self.pub_way_toggle.publish(True)
            self.pub_lane_toggle.publish(False)
        elif self.mode_flag == 3:   # 주행 종료
            self.stop_drive()
        else:
            self.stop_drive()


    def stop_drive(self):
        self.pub_obs_toggle.publish(False)
        self.pub_way_toggle.publish(False)
        self.pub_lane_toggle.publish(False)
        self.drive.linear.x = 0
        self.drive.angular.z = 0
        self.pub_drive.publish(self.drive)
        

if __name__ == '__main__':
    try:
        main_jeju()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program terminated")
