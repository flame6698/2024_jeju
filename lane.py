#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 이전 점의 위치를 저장하는 변수
last_line_center_left = None
last_line_center_right = None
main_toggle = False  # 'main_cmd_lane' 토픽의 상태를 추적하는 변수

def main_cmd_lane_callback(data):
    global main_toggle
    main_toggle = data.data

def image_callback(img_msg):
    global last_line_center_left, last_line_center_right, main_toggle

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    ksize = 17  # 가우시안 블러 커널 사이즈 (홀수)
    canny_low = 68  # 캐니 엣지 검출 최소 임곗값
    canny_high = 100  # 캐니 엣지 검출 최대 임곗값

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (ksize, ksize), 0)
    edges = cv2.Canny(blurred, canny_low, canny_high)
    height, width = cv_image.shape[:2]

    roi_height = height // 10
    roi = edges[height - roi_height:height, :]

    center_x = width // 2
    line_center_left = 0  # 왼쪽 점 초기화
    line_center_right = width  # 오른쪽 점 초기화
    # max_angular_z = 0.3  # 최대 각속도

    max_white_count_left = 0
    max_white_count_right = 0
    linear_value = 1

    # 왼쪽 영역에서 검출
    for x in range(0, center_x, 20):
        white_count = cv2.countNonZero(roi[:, x:x+20])
        if white_count > max_white_count_left:
            max_white_count_left = white_count
            line_center_left = x + 10

    # 오른쪽 영역에서 검출
    for x in range(center_x, width, 20):
        white_count = cv2.countNonZero(roi[:, x:x+20])
        if white_count > max_white_count_right:
            max_white_count_right = white_count
            line_center_right = x + 10

    # 이전 값과 비교하여 큰 변화가 있는 경우 이전 값 유지
    if last_line_center_left is not None and abs(line_center_left - last_line_center_left) > 150:
        line_center_left = last_line_center_left
    else:
        last_line_center_left = line_center_left

    if last_line_center_right is not None and abs(line_center_right - last_line_center_right) > 150:
        line_center_right = last_line_center_right
    else:
        last_line_center_right = line_center_right

    # 조향 rad/s 계산
    # 최대 155까지 차이날 수 있음
    # 선속도가 결정나면, 곡률반경에 따라 최대 angular_z값이 결정이 나는데 이 값이 최대가 되는 상수를 나눠줌
    # 155 / 515 = 0.301.. 그런데 조향이 급한곳에서는 미리 보고 명령을 줘야 코스를 안정적으로 돌아서 숫자를 작게해서 각도값의 최대값이 좀 일찍 나오게 함
    angular_z = (center_x - float(line_center_left + line_center_right) / 2) / 5
    twist = Twist()
    twist.linear.x = 0.6 * linear_value  # Set a constant linear speed
    twist.angular.z = angular_z

    # 'main_toggle'이 True일 때만 명령 전송
    if main_toggle:
        cmd_vel_pub.publish(twist)

    # 결과 이미지에 원을 그리는 로직과 OpenCV 이미지 출력 로직
    # 이 부분도 이전과 동일하므로 생략합니다.

rospy.init_node('lane_center_publisher', anonymous=True)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
main_cmd_sub = rospy.Subscriber('main_cmd_lane', Bool, main_cmd_lane_callback)  # 'main_cmd_lane' 토픽을 구독하는 라인 추가
bridge = CvBridge()

rospy.spin()
