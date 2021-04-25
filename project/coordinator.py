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
           (9,-6.5),(9,-0.5),(4.5,-0.5)]

class PatrolCoordinator(object): 
    def __init__(self, availableRobots=['robot1', 'robot2', 'robot3'], listOfPositions={'robot1':[], 'robot2':[], 'robot3':[]}): #-6.26
        rospy.loginfo('coordinator initialization')
        self.task_list={} # requirements
        self.nextPositionIndex = {'robot1':0, 'robot2':0, 'robot3':0}
        self.availableRobots = availableRobots
        self.listOfPositions = {}
        self.status_subscriber={}
        self.robotPublishers = {}
        self.listOfPositions = listOfPositions
        #self.cur_goal={}
        self.task={}
        for i in node_list:
            self.task_list[i]=0
        for robotName in availableRobots:
            
            self.status_subscriber[robotName] = rospy.Subscriber(robotName+'/patrol_status', String, self.askForNextLocation)
            topic = robotName+'/coordinator_directions'
            self.robotPublishers[robotName] = rospy.Publisher(topic, Pose2D, queue_size=10, latch=True)
            #self.cur_goal[robotName]=None
            self.task[robotName]=[]
        self.tasklen=len(self.task_list)
    def askForNextLocation(self, msg=String("")):
        # get the name of the node that is sending the message
        robotName = msg.data[:6]
        if msg.data[6:] == "failure":
            rospy.logerr("failure by patrolling")
        # else:
        if len(self.task[robotName]):
            if self.listOfPositions[robotName][0]==self.task[robotName][0]: # finish the task
                if len(self.task[robotName])>2:
                    print('-----------task completed------------')
                    print(self.task_list)
                    print(self.task[robotName][1],self.task[robotName][0])
                    self.task_list[self.task[robotName][1]]+=self.task_list[self.task[robotName][0]]

                self.task_list[self.task[robotName][0]]=0
                print(self.task_list)
                self.task[robotName].pop(0) # delete the task
        if len(self.listOfPositions[robotName]):
            self.listOfPositions[robotName].pop(0)
        self.sendDirection(robotName)
        
    
    def sendDirection(self, robot):
        #print(self.nextPositionIndex[robot],len(self.listOfPositions[robot]))
        if len(self.listOfPositions[robot])==0:
            return
        pub = self.robotPublishers[robot]
        #positionIndex = self.nextPositionIndex[robot]
        position = self.listOfPositions[robot][0]
        direction = Pose2D(position[0], position[1], 0)
        pub.publish(direction)
        #self.nextPositionIndex[robot] = self.nextPositionIndex[robot] + 1  # need to modify

    def renew(self, routes, tasks, name):
        routes.pop(0)
        index=0
        name='robot'+str(name)
        num=len(self.listOfPositions[name])
        for i in range(len(self.listOfPositions[name])):
            if self.listOfPositions[name][i]==routes[0]:
                index=i
                break
        new_pos=self.listOfPositions[name][:index]+routes
        self.listOfPositions[name]=new_pos
        tasks.pop(0)
        self.task[name]=tasks
        if num==0:
            self.sendDirection(name)



def main():
    rospy.init_node('coordinator')
    coordinator = PatrolCoordinator(availableRobots=['robot1', 'robot2', 'robot3'])
    
    coordinator.sendDirection('robot1')
    coordinator.sendDirection('robot2')
    coordinator.sendDirection('robot3')
    rospy.spin()

if __name__ == '__main__':
    main()
