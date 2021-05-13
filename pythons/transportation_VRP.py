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
random.seed(1) # 控制变量

renew_int=10
num_of_new=2
capacity=100


def order_init(coordinator, my_vrp): # 订单初始化
    vacant=[] # 没有订单的任务点
    for i in list(coordinator.task_list.keys()):
        if coordinator.sample_flag[i]==1 and i!=(0,0):
            vacant.append(i)
    new=random.sample(vacant,10) # 随机产生10个订单

    for i in new:
        coordinator.task_list[i]=random.randint(20,23) # 随机产生需求量
        coordinator.sample_flag[i]=0

    my_vrp.input_demand_and_car_num(coordinator.task_list,3,[],[],[1,2,3]) # DVRP
    routes, tasks, robot=my_vrp.cal_everything() # 得到机器人对应的任务与路径

    for i in range(len(routes)):
        coordinator.renew(routes[i],tasks[i],robot[i]) # 更新任务与路径

def order_generator(): # 产生新订单
    vacant=[]

    for i in list(coordinator.task_list.keys()):
        if coordinator.task_list[i]==0 and i!=(0,0):
            vacant.append(i)

    new=random.sample(vacant,min(3,len(vacant))) # 从没有订单的任务点产生新订单
    for i in new:
        coordinator.task_list[i]=random.randint(10,20) # 随机需求量
        coordinator.sample_flag[i]=0 # 已经有订单


if __name__=='__main__':
    rospy.init_node('transportation', anonymous=True)
    my_vrp=VRP(16, capacity) # 初始化DVRP
    actionExecuter1 = ActionExec('robot1') #初始化机器人
    actionExecuter2 = ActionExec('robot2')
    actionExecuter3 = ActionExec('robot3')
    coordinator = PatrolCoordinator(availableRobots=['robot1', 'robot2', 'robot3']) # 初始化调度中心
    print('-------------round 0--------------')
    order_init(coordinator, my_vrp) # 订单初始化

    rospy.sleep(renew_int)
    round=1
    counter=0
    while True: # 每隔一段时间产生新订单
        if (counter+1)%6==0:
            print(' ')
            print('-------------round '+str(round)+'--------------')
            print('We have finished altogether '+str(coordinator.finished)+' tasks now.')
            order_generator() # 产生新订单
            round+=1

        counter+=1

        goal_list=[]
        name=['robot'+str(j+1) for j in range(3)]
        name_list=[]
        for i in name: # 找做完任务的机器人
            if len(coordinator.listOfPositions[i])==0:
                name_list.append(int(i[-1]))

        if len(name_list)>0: # 有做完任务的机器人
            my_vrp.input_demand_and_car_num(coordinator.task_list,len(name_list),[],[],name_list) # DVRP
            routes, tasks, robot=my_vrp.cal_everything() # 更新任务与路径

            for i in range(len(routes)): # 更新机器人
                coordinator.renew(routes[i],tasks[i],robot[i])

        rospy.sleep(renew_int)

