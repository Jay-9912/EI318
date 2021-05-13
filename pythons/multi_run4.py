# -*- coding: utf-8 -*-

import rospy
import threading



class myThread (threading.Thread): # 通过多线程实现多机器人并行控制
  def __init__(self, threadID, name, point):
      threading.Thread.__init__(self)
      self.navigation = GoToPose(name)
      self.name = name
      self.point = point
  def run(self):
    Nav_state = self.navigation.Point_Navigation(self.point)




if __name__ == '__main__':
  A_1st = {"name":"A","pose":(4.0, -6.26, 0.0),"orientation":(0.0, 0.0, 0.0, 1.0),"object":2}
  B_1st = {"name":"A","pose":(0.0, -6.26, 0.0),"orientation":(0.0, 0.0, 0.0, 1.0),"object":2}
  A_2nd = {"name":"F","pose":(4.0, 5.67, 0.0),"orientation":(0.0, 0.0, 0.0, 1.0),"object":0}
  d2 = {"name":"F","pose":(0.0, 5.65, 0.0),"orientation":(0.0, 0.0, 0.0, 1.0),"object":0}
  rospy.init_node('robot1_display', anonymous=False)

  thread1 = myThread(1, "robot1", B_1st)
  thread2 = myThread(2, "robot2", d2)
  

  thread1.start()
  thread2.start()
  thread1.join()
  thread2.join()
  rospy.sleep(1)






  
