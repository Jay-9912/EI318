import os
import rospy
import actionlib
from std_msgs.msg import String
import random
from geometry_msgs.msg import Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from coordinator_VRP import *
from multi_run3 import *
import random
import math
from VRP import *


renew_int=10
num_of_new=2
capacity=100

def order_init(coordinator, my_vrp):
    vacant=[]
    for i in list(coordinator.task_list.keys()):
        if coordinator.sample_flag[i]==1 and i!=(0,0):
            vacant.append(i)
    new=random.sample(vacant,10)
    print(new)
    for i in new:
        coordinator.task_list[i]=random.randint(20,23)
        coordinator.sample_flag[i]=0
    #print(coordinator.task_list)
    print('task_list:')
    print(coordinator.task_list)
    my_vrp.input_demand_and_car_num(coordinator.task_list,3,[],[],[1,2,3])
    routes, tasks, robot=my_vrp.cal_everything() # need robot name?
    print('routes:')
    print(routes)
    print('tasks:')
    print(tasks)
    print('robots:')
    print(robot)
    for i in range(len(routes)):
        coordinator.renew(routes[i],tasks[i],robot[i])

def order_generator():
    vacant=[]
    print('sample')
    print(coordinator.sample_flag)
    for i in list(coordinator.task_list.keys()):
        if coordinator.task_list[i]==0 and i!=(0,0):
            vacant.append(i)

    new=random.sample(vacant,min(3,len(vacant)))
    for i in new:
        coordinator.task_list[i]=random.randint(10,20)
        coordinator.sample_flag[i]=0
    print('sample')
    print(coordinator.sample_flag)

if __name__=='__main__':
    rospy.init_node('transportation', anonymous=True)
    my_vrp=VRP(16, capacity)
    actionExecuter1 = ActionExec('robot1')
    actionExecuter2 = ActionExec('robot2')
    actionExecuter3 = ActionExec('robot3')
    coordinator = PatrolCoordinator(availableRobots=['robot1', 'robot2', 'robot3'])
    order_init(coordinator, my_vrp)
    #coordinator.sendDirection('robot1')
    #coordinator.sendDirection('robot2')
    #coordinator.sendDirection('robot3')
    rospy.sleep(renew_int)
    round=1
    counter=0
    while True:
        if (counter+1)%6==0:
            print('-------------round'+str(round)+'--------------')
            order_generator()
            round+=1

        counter+=1


        goal_list=[]
        name=['robot'+str(j+1) for j in range(3)]
        name_list=[]
        for i in name:
            if len(coordinator.listOfPositions[i])==0:
                #goal_list.append(coordinator.task[i][0])
                name_list.append(int(i[-1]))

        #t_list=list(coordinator.task_list.values())
        #num_of_car=math.ceil(sum(t_list)/capacity)

        #print('num_of_car:',num_of_car)
        #print('goal_list:')
        #print(goal_list)

        if len(name_list)>0:
            print('task_list:')
            print(coordinator.task_list)

            print('name_list:')
            print(name_list)
            my_vrp.input_demand_and_car_num(coordinator.task_list,len(name_list),[],[],name_list)
            routes, tasks, robot=my_vrp.cal_everything() # need robot name?
            print('routes:')
            print(routes)
            print('tasks:')
            print(tasks)
            print('robots:')
            print(robot)
            for i in range(len(routes)):
                coordinator.renew(routes[i],tasks[i],robot[i])
                # for j in name:
                #     if routes[i][1]==coordinator.task[j][0]:
                #         coordinator.renew(routes, j)
                #         break
        rospy.sleep(renew_int)


    #rospy.spin()
