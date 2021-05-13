#!/usr/bin/env python
import os
import rospy
import actionlib
import pdb
import argparse


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class ActionExec(object):
    def __init__(self,  robot, client_topic='/move_base'):
        self.client_topic = client_topic
        self.name=robot
        self.status_publisher = rospy.Publisher(self.name+'/patrol_status', String, queue_size=10) # 发布状态
        self.directions_subscriber = rospy.Subscriber(self.name+'/coordinator_directions', Pose2D, self.sendARobot) # 订阅目的地坐标
        self.client = actionlib.SimpleActionClient(self.name+self.client_topic, MoveBaseAction) # 初始化Client，关联相应topic
        self.client.wait_for_server(rospy.Duration(5))
        
        rospy.logdebug('initialized action exec')
    
    def sendARobot(self, msg):
        pose = msg
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id=self.name+'_tf/map'
        

        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0   
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        self.client.send_goal(goal) # 去目的地
        finishedOnTime = self.client.wait_for_result(rospy.Duration(300))
        state = self.client.get_state()

        if state == 3: # 反馈状态
            self.status_publisher.publish(self.name+"success")
        else:
            rospy.logerr("navigation failed")
            self.status_publisher.publish(self.name+"failure")
            
    
def main():
    parser = argparse.ArgumentParser()
    rospy.init_node('client_patrol', anonymous=True)
    actionExecuter1 = ActionExec('robot1')
    actionExecuter2 = ActionExec('robot2')
    actionExecuter3 = ActionExec('robot3')
    rospy.spin()
    
if __name__ == '__main__':
    main()
