#!/usr/bin/env python
import re
import rospy
from std_msgs.msg import String
import random
from geometry_msgs.msg import Pose2D
import pdb
import argparse

node_list=[(0,0),(9,6.5),(4.5,6.5),(0,6.5),(-4.5,6.5),(-4.5,-1.5),(-10.5,-1.5),\
           (-7.5,-6.5),(0,-6.5),(0,-12.5),(4.5,-12.5),(4.5,-6.5),(9,-12.5),\
           (9,-6.5),(9,-0.5),(4.5,-0.5)]  # 任务地点

class PatrolCoordinator(object): 
    def __init__(self, availableRobots=['robot1', 'robot2', 'robot3'], listOfPositions={'robot1':[], 'robot2':[], 'robot3':[]}): #-6.26
        rospy.loginfo('coordinator initialization')
        self.task_list={} # 任务点对应的需求量
        self.nextPositionIndex = {'robot1':0, 'robot2':0, 'robot3':0} # 机器人清单
        self.availableRobots = availableRobots # 机器人状态订阅器
        self.status_subscriber={} # 机器人状态订阅器
        self.robotPublishers = {} # 机器人状态发布器
        self.listOfPositions = listOfPositions # 机器人将要走的路径
        self.sample_flag={} # 是否可以重新产生需求的标志
        self.finished = 0 # 目前完成的任务总量
        self.task={} # 机器人的任务清单
        for i in node_list:
            self.task_list[i]=0
            self.sample_flag[i]=1
        for robotName in availableRobots:
            
            self.status_subscriber[robotName] = rospy.Subscriber(robotName+'/patrol_status', String, self.askForNextLocation) # 订阅状态
            topic = robotName+'/coordinator_directions'
            self.robotPublishers[robotName] = rospy.Publisher(topic, Pose2D, queue_size=10, latch=True) # 目的地发布器
            self.task[robotName]=[]

    def askForNextLocation(self, msg=String("")):
        robotName = msg.data[:6] # 获得当前机器人的名字
        if msg.data[6:] == "failure":
            rospy.logerr("failure by patrolling")
        if len(self.task[robotName]):
            if self.listOfPositions[robotName][0]==self.task[robotName][0]: # 已经在任务点，完成任务
                print(robotName+' finishes one of its tasks!')
                self.finished += 1 # 计数
                self.sample_flag[self.task[robotName][0]]=1 # 更新标志
                self.task[robotName].pop(0) # 删除已完成的任务
        if len(self.listOfPositions[robotName]):
            self.listOfPositions[robotName].pop(0) # 删除经过的路径点
        self.sendDirection(robotName) # 去下一路径点
        
    
    def sendDirection(self, robot):
        if len(self.listOfPositions[robot])==0: # 所有任务完成，停止运动
            return
        pub = self.robotPublishers[robot]
        position = self.listOfPositions[robot][0]
        direction = Pose2D(position[0], position[1], 0)
        pub.publish(direction) # 发布目的地

    def renew(self, routes, tasks, name): # 更新任务清单和路径
        routes.pop(0)
        name='robot'+str(name)
        self.listOfPositions[name]=routes
        tasks.pop(0)
        self.task[name]=tasks
        for i in tasks:
            self.task_list[i]=0
        self.sendDirection(name) # 重新出发



def main():
    rospy.init_node('coordinator')
    coordinator = PatrolCoordinator(availableRobots=['robot1', 'robot2', 'robot3'])

    coordinator.sendDirection('robot1')
    coordinator.sendDirection('robot2')
    coordinator.sendDirection('robot3')
    rospy.spin()

if __name__ == '__main__':
    main()
