import os
import rospy
import actionlib
from std_msgs.msg import String
import random
from geometry_msgs.msg import Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from coordinator import *
from multi_run3 import *
import random
import math
from DVRP import *
random.seed(1) # 控制变量

renew_int=60
num_of_new=2
capacity=100

def order_init(coordinator, my_dvrp): # 订单初始化
    vacant=[] # 没有订单的任务点
    for i in list(coordinator.task_list.keys()):
        if coordinator.task_list[i]==0:
            vacant.append(i)
    new=random.sample(vacant,10) # 随机产生10个订单

    for i in new:
        coordinator.task_list[i]=random.randint(20,23) # 随机产生需求量

    my_dvrp.input_demand_and_car_num(coordinator.task_list,3,[],[]) # DVRP
    routes, tasks, robot=my_dvrp.cal_everything() # 得到机器人对应的任务与路径

    for i in range(len(routes)):
        coordinator.renew(routes[i],tasks[i],robot[i]) # 更新任务与路径

def order_generator(): # 产生新订单
    vacant=[] 
    for i in list(coordinator.task_list.keys()): 
        if coordinator.task_list[i]==0:
            vacant.append(i)
    new=random.sample(vacant,3) # 从没有订单的任务点产生新订单
    for i in new:
        coordinator.task_list[i]=random.randint(10,20) # 随机需求量

if __name__=='__main__':
    rospy.init_node('transportation', anonymous=True)
    my_dvrp=DVRP(16, capacity) # 初始化DVRP
    actionExecuter1 = ActionExec('robot1') #初始化机器人
    actionExecuter2 = ActionExec('robot2')
    actionExecuter3 = ActionExec('robot3')
    coordinator = PatrolCoordinator(availableRobots=['robot1', 'robot2', 'robot3']) # 初始化调度中心
    print('-------------round 0--------------')
    order_init(coordinator, my_dvrp) # 订单初始化

    rospy.sleep(renew_int) 
    round=1
    while True: # 每隔一段时间产生新订单
        print(' ')
        print('-------------round '+str(round)+'--------------')
        print('We have finished altogether '+str(coordinator.finished)+' tasks now.')
        order_generator() # 产生新订单
        round+=1
        goal_list=[]
        name=['robot'+str(j+1) for j in range(3)]
        name_list=[]
        for i in name: # 找在做任务的机器人
            if len(coordinator.task[i])>1 and coordinator.task[i][0]!=(0,0):
                goal_list.append(coordinator.task[i][0])
                name_list.append(int(i[-1]))

        my_dvrp.input_demand_and_car_num(coordinator.task_list,3,goal_list,name_list) # DVRP
        routes, tasks, robot=my_dvrp.cal_everything() # 得到任务与路径

        task_num = 0 # 当前剩下多少任务
        for i in range(len(routes)):
            task_num += len(tasks[i]) - 1
            coordinator.renew(routes[i],tasks[i],robot[i]) # 更新任务与路径

        print('There are altogether '+str(task_num)+' tasks to finish now.')
        rospy.sleep(renew_int)


