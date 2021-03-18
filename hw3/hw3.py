#! /usr/bin/env python
# -*- coding: utf-8 -*-

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf import transformations
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from math import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import math

pub_ = None
regions_ = {   # 将视野分为五个片区，用列表记录每个片区内离障碍物的距离
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0   # 机器人当前状态
state_dict_ = {       # 状态字典
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
err_sum = 0   # 距离误差积分项

which_task = -1  # 任务标志，未检测到为-1，红色为0，蓝色为1，绿色为2
red_suc1 = 0
red_suc2 = 0
blue_start_turn = 0
blue_start_straight = 0
turn_error = 0.001
turn_sum_error = 0
ver_dis = 0.15


color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              }  # 三种颜色的阈值
class Image_converter:
    def __init__(self):

	self.bridge = CvBridge()
	
	self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
    
    	# Allow up to one second to connection
        rospy.sleep(1)

    def callback(self,data):
		global which_task
		if which_task != -1:
		    return
		# Convert image to OpenCV format，
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
		    print e
		colors = ['red', 'blue', 'green']

		for i, c in enumerate(colors):   # 依次检测三种颜色
		    color_detected = self.detect_table(cv_image, c)
		    if color_detected:
                	which_task = i
                	rospy.loginfo("find color: %s", c)
                	return



    def detect_table(self,image,c):
	
		g_image = cv2.GaussianBlur(image, (5, 5), 0)	# 预处理	
		hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)  # 转为hsv        
                erode_hsv = cv2.erode(hsv, None, iterations=2)   # 腐蚀               
                inRange_hsv = cv2.inRange(erode_hsv, color_dist[c]['Lower'], color_dist[c]['Upper'])  # 检测
                contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # 找出边沿 
		return len(contours) > 0

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))  # 机器人的位姿：位置与四元数

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
	
def clbk_laser(msg):
    global regions_
    global which_task
    global red_suc2

    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }  # 视角分为五个片区
    if which_task == -1:
    	take_action()
    if which_task == 0 and red_suc2:
        # 设置画布为600*600像素
        frame = np.zeros((600, 600,3), np.uint8)
        newdata = msg
        angle = msg.angle_min
        newdata.ranges = list(msg.ranges)

        cnt = 0
        for i in range(len(newdata.ranges)):
            if cnt == 0:
                if newdata.ranges[i+1] < newdata.ranges[i]:
                    cnt = 1
                    start_point = i
            if cnt == 1:
                if (newdata.ranges[i + 1] - newdata.ranges[i]) > 1:
                    cnt = 2
                    last_point = i
                    break

        theta = (last_point - start_point) * 0.0016367#好像读出的参数data.angle_increment 与显示(0.0043..)的不同
        #余弦定理计算床的长度
        length = math.sqrt(newdata.ranges[start_point] ** 2 + newdata.ranges[last_point] ** 2
                       - 2 * newdata.ranges[start_point] * newdata.ranges[last_point] * math.cos(theta))

        print('the length of bed is', length)#输出床的长度

        # change infinite values to 0
        for r in msg.ranges:
            if math.isinf(r) == True:
                r = 0

            # convert angle and radius to cartesian coordinates
            x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
            y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))

            # set the borders (all values outside the defined area should be 0)
            if y > 600 or y < -600 or x<-600 or x>600:
                x=0
                y=0
            # print "xy:",x,y
            # 用CV2画线，位置在(300,300),和目标点，颜色是(255,0,0),线宽2
            cv2.line(frame,(300, 300),(x+300,y+300),(255,0,0),2)
            # 角度的增加
            angle= angle + msg.angle_increment
            # 画个中心圆
            cv2.circle(frame, (300, 300), 2, (255, 255, 0))
            cv2.imshow('frame',frame)
            cv2.waitKey(1)

    global  blue_start_turn
    global  blue_start_straight
    global  turn_error
    global  ver_dis
    if  which_task == 1 and blue_start_turn:

        turn_error = sum(msg.ranges[352:357])-sum(msg.ranges[358:363])
        print turn_error

    if which_task == 1 and blue_start_straight:

        ver_dis = msg.ranges[357]





def change_state(state):
    global state_, state_dict_
    if state is not state_:  # 状态变化
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 0.6  # 距离阈值

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:  # 三个方向均无障碍，向右前方找墙
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d: # 前方有障碍，左转绕行
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d: # 右前方有障碍，沿墙走
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d: # 左前方有障碍，向右前方找墙
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d: # 前方和右前方有障碍，左转
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d: # 前方和左前方有墙，左转
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d: # 三个方向都有障碍，左转
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d: # 左前方和右前方有障碍，向右前方找墙
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    rospy.loginfo(state_description)

def find_wall(): # 向右前方找墙
    msg = Twist()
    msg.linear.x = 0.2 # 向前
    msg.angular.z = -0.3 # 向右
    return msg

def turn_left(): # 左转
    msg = Twist() 
    msg.angular.z = 0.3 # 向左
    return msg

def follow_the_wall(): # 沿墙走
    global regions_
    global err_sum
    obj_dist = 0.6 # 要控制的离墙距离
    err = regions_['right'] - obj_dist # 离目标量的距离，比例项
    coefp = -2.5 # P系数
    coefi = -1.0 # I系数
    err_sum = err_sum + 0.01 * err # 积分项
    ctrl = coefp * err + coefi * err_sum  # 控制量
    msg = Twist()
    msg.angular.z = constrain(ctrl, -0.2, 0.2) # 限定角速度范围
    msg.linear.x = 0.5 
    return msg

def constrain(val, lower, upper): # 限定上下限
    if val > upper:
        return upper
    return val if val > lower else lower


def task_red():
    global red_suc1
    global red_suc2
    navigator = GoToPose()
    theta_x=0
    theta_y=0
    theta_z=1.05#30
    r1=math.cos(theta_x/2)*math.cos(theta_y/2)*math.cos(theta_z/2) + math.sin(theta_x/2)*math.sin(theta_y/2)*math.sin(theta_z/2)

    r2=math.sin(theta_x/2)*math.cos(theta_y/2)*math.cos(theta_z/2) - math.cos(theta_x/2)*math.sin(theta_y/2)*math.sin(theta_z/2)
    
    r3=math.cos(theta_x/2)*math.sin(theta_y/2)*math.cos(theta_z/2) + math.sin(theta_x/2)*math.cos(theta_y/2)*math.sin(theta_z/2)

    r4=math.cos(theta_x/2)*math.cos(theta_y/2)*math.sin(theta_z/2) - math.sin(theta_x/2)*math.sin(theta_y/2)*math.cos(theta_z/2)

    position = {'x': 10.9, 'y': -3.51}
    quaternion = {'r1': 0, 'r2': 0, 'r3': 0, 'r4': 1}
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    red_suc1 = navigator.goto(position, quaternion)

    if red_suc1:
        position = {'x': 9.92, 'y': -6.74}
        quaternion = {'r1': 0, 'r2': 0, 'r3': -0.5, 'r4': math.sqrt(3) / 2}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        red_suc2 = navigator.goto(position, quaternion)

    pass

def task_blue():
    try:
        global blue_start_turn
        navigator = GoToPose()
        position = {'x': 12.8, 'y' : 2.3}

        quaternion = {'r1': 0, 'r2': 0, 'r3': 0, 'r4': 1}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        blue_start_turn = navigator.goto(position, quaternion)
           #blue_start_turn = navigator.goto(position, quaternion)
        #blue_start_turn = 1
        if blue_start_turn:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        global turn_error
        global turn_sum_error

        blue_method = 0
        turn_flag = 0
        rate = rospy.Rate(20)
        while turn_flag == 0 :
                if blue_method == 0:
                    coefp = -2.5
                    coefi = -1.0
                    turn_sum_error = turn_sum_error + 0.01 * turn_error
                    ctrl = coefp * turn_error + coefi * turn_sum_error
                    msg = Twist()
                    msg.angular.z = constrain(ctrl, -0.035, 0.035)
                    msg.linear.x = 0
                    #print turn_error

                    if turn_error <= 0.0001:
                        turn_flag = 1
                        msg = Twist()
                        msg.angular.z = 0

                    pub_.publish(msg)
                    rate.sleep()

                #if blue_method == 1 :

        blue_start_turn = 0
        rospy.loginfo("Turn_SUCCESS")

        straight_flag = 0
        global ver_dis
        global blue_start_straight
        blue_start_straight = 1
        while  straight_flag == 0:

            msg = Twist()
            msg.linear.x = 0.05
            pub_.publish(msg)
            if ver_dis <=0.1:
                straight_flag = 1
                blue_start_straight = 0
                msg.linear.x = 0
                pub_.publish(msg)
                time.sleep(5)

            

        rospy.loginfo("Straight_SUCCESS")

        position = {'x': 11.9, 'y' : -1.58}
        last_flag = navigator.goto(position, quaternion)

        if last_flag:
            rospy.loginfo("Blue_task_finish")
        else:
            rospy.loginfo("Leave the room failed")

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

def task_green():
    try:
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        # trash bin at (16.4, -6.51)  d = 0.42
	for i in range(6): # 在垃圾桶附近找四个点，绕一圈后回到门口
        if i == 0:
	        position = {'x': 15.4, 'y' : -6.5} 
	    elif i == 1:
            position = {'x': 16.1, 'y' : -7.38} 
        elif i == 2:
	        position = {'x': 17.3, 'y' : -6.71}
	    elif i == 3:
            position = {'x': 16.7, 'y' : -5.71} 
	    elif i == 4:
            position = {'x': 15.4, 'y' : -6.5} 
        elif i == 5:
	        position = {'x': 11.9, 'y' : -1.58}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion) # 是否到达目标位置

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
	

def main():
    global pub_
    global which_task
    rospy.init_node('reading_laser')

    Image_converter()
    which_task = 1
    pub_ = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

    sub = rospy.Subscriber('/turtlebot/laser/scan', LaserScan, clbk_laser)

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown() and which_task == -1: # 未检测到任务则沿右墙找任务
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()
    if which_task == 0:
        task_red()
    elif which_task == 1:
        task_blue()
    elif which_task == 2:
        task_green()
    rospy.spin()
if __name__ == '__main__':
    main()



