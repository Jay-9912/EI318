import numpy as np
from operator import itemgetter
import copy


global route_path

class DVRP():

    def __init__(self,place_num,ton):

        self.place_num = place_num  
        self.tons = ton 
        self.distance = np.load('D.npy')  
        self.demand = []  
        self.savings = [] 
        self.Routes = []   
        self.car_num = 0
        self.first_place_list = []
        self.first_robot_index = []
        self.P_matrix = np.load('P.npy')
        self.route_list = []
        self.node_list =[(0,0), (9,6.5),(4.5,6.5),(0,6.5),(-4.5,6.5),(-4.5,-1.5),(-10.5,-1.5),\
           (-7.5,-6.5),(0,-6.5),(0,-12.5),(4.5,-12.5),(4.5,-6.5),(9,-12.5),\
           (9,-6.5),(9,-0.5),(4.5,-0.5)]
        self.task_list = []
        self.robot_route_list = []
        self.route_index = []

        route_list = []
        for i in range(len(self.node_list)):
            each_list = []
            for j in range(len(self.node_list)):
                path_list = []
                path_list.append((self.node_list[i][0], self.node_list[i][1]))
                temp = self.P_matrix[i][j]
                path_list.append((self.node_list[temp][0], self.node_list[temp][1]))
                while (temp != j):
                    temp = self.P_matrix[temp][j]
                    path_list.append((self.node_list[temp][0], self.node_list[temp][1]))
                each_list.append(path_list)
            route_list.append(each_list)
        self.route_list = route_list


    def input_demand_and_car_num(self,demand_list,car_num,first_place,robot_index): 
        # print('dvrp task_list:')
        # print(demand_list)
        # print('dvrp first place:')
        # print(first_place)
        # print('dvrp robot_index')
        # print(robot_index)
        self.demand = [0 for i in range(len(demand_list.keys()))]
        for i in demand_list.keys():
            self.demand[self.node_list.index(i)] = demand_list[i]
        self.demand[0]=0
        self.car_num = car_num
        self.first_robot_index = robot_index
        self.first_place_list = []
        for i in range(len(first_place)):
            self.first_place_list.append(self.node_list.index(first_place[i]))
        self.task_list=[]
        self.robot_route_list=[]
        self.route_index=[]
        self.savings=[]
        self.Routes=[]
    def savingsAlgorithms(self):
        saving = 0
        valid_list = []
        for i in range(1, len(self.demand)):

             if self.demand[i]!=0:
                self.Routes.append([i])
                valid_list.append(i)


        for i in valid_list:                                        
            for j in valid_list:
                if i == j:
                    pass
                else:
                    saving = (self.distance[i][0] + self.distance[0][j]) - self.distance[i][j]
                    self.savings.append([i, j, saving])                                   

        self.savings = sorted(self.savings, key=itemgetter(2), reverse=True)                   


        for i in range(len(self.savings)):
            startRoute = []
            endRoute = []
            routeDemand = 0
            for j in range(len(self.Routes)):
                if (self.savings[i][0] == self.Routes[j][-1]):
                    endRoute = self.Routes[j]
                elif (self.savings[i][1] == self.Routes[j][0]):
                    startRoute = self.Routes[j]

                if ((len(startRoute) != 0) and (len(endRoute) != 0)):
                    for k in range(len(startRoute)):
                        routeDemand += self.demand[startRoute[k]]
                    for k in range(len(endRoute)):
                        routeDemand += self.demand[endRoute[k]]

                    if (routeDemand <= self.tons):
                       if startRoute[0] not in self.first_place_list:
                          self.Routes.remove(startRoute)
                          self.Routes.remove(endRoute)
                          self.Routes.append(endRoute+startRoute)
                       elif endRoute[0] not in self.first_place_list:
                           self.Routes.remove(startRoute)
                           self.Routes.remove(endRoute)
                           self.Routes.append(startRoute+endRoute)
                    break

            if len(self.Routes) == self.car_num:
                break

        for i in range(len(self.Routes)):
            self.Routes[i].insert(0, 0)
            self.Routes[i].insert(len(self.Routes[i]), 0)


    def printRoutes(self):

            for i in self.Routes:
                costs = 0
                capacity = 0
                for j in range(len(i) - 1):
                    costs += self.distance[i[j]][i[j + 1]]
                    capacity += self.demand[i[j]]
                    #print(self.distance[i[j]][i[j + 1]])



    def check_capacity_leagal(self,rand1,rand2,rand3,rand4):
        if rand1 == rand2:
            return 1
        else:
            if self.demand[self.Routes[rand1][rand3]] > self.demand[self.Routes[rand2][rand4]]:
                capacity = 0
                for i in range(len(self.Routes[rand2])):
                    capacity += self.demand[self.Routes[rand2][i]]

                capacity = capacity - self.demand[self.Routes[rand2][rand4]] + self.demand[self.Routes[rand1][rand3]]
                if capacity > self.tons:
                    return 0
                else:
                    return 1
            else:
                capacity = 0
                for i in range(len(self.Routes[rand1])):
                    capacity += self.demand[self.Routes[rand1][i]]

                capacity = capacity - self.demand[self.Routes[rand1][rand3]] + self.demand[self.Routes[rand2][rand4]]
                if capacity > self.tons:
                    return 0
                else:
                    return 1


    def exchange_and_relocate(self,action_num):  

            leagal_flag = 0
            while (leagal_flag == 0):
                rand = np.random.randint(0, 2)  

                if rand == 0:
                    rand1 = np.random.randint(0, len(self.Routes))
                    rand2 = np.random.randint(0, len(self.Routes))
                    rand3 = np.random.randint(1, len(self.Routes[rand1]) - 1)
                    rand4 = np.random.randint(1, len(self.Routes[rand2]) - 1)
                else:
                    rand1 = np.random.randint(0, len(self.Routes))
                    rand2 = rand1
                    rand3 = np.random.randint(1, len(self.Routes[rand1]) - 1)
                    rand4 = np.random.randint(1, len(self.Routes[rand2]) - 1)


                if (rand1 == rand2 and abs(rand3 - rand4) <= 1 ) or (
                        self.Routes[rand1][rand3] in self.first_place_list ) or (
                        self.Routes[rand2][rand4] in self.first_place_list ) or (self.check_capacity_leagal(rand1, rand2, rand3, rand4) == 0):
                    continue

                leagal_flag = 1


            if rand == 0:
                new_route = copy.deepcopy(self.Routes)
                tmp = new_route[rand1][rand3]
                new_route[rand1][rand3] = new_route[rand2][rand4]
                new_route[rand2][rand4] = tmp
                old_distance = self.distance[self.Routes[rand1][rand3 - 1]][self.Routes[rand1][rand3]] + \
                               self.distance[self.Routes[rand1][rand3]][self.Routes[rand1][rand3 + 1]] \
                               + self.distance[self.Routes[rand2][rand4 - 1]][self.Routes[rand2][rand4]] + \
                               self.distance[self.Routes[rand2][rand4]][self.Routes[rand2][rand4 + 1]]
                new_distance = self.distance[self.Routes[rand1][rand3 - 1]][self.Routes[rand2][rand4]] + \
                               self.distance[self.Routes[rand2][rand4]][self.Routes[rand1][rand3 + 1]] \
                               + self.distance[self.Routes[rand2][rand4 - 1]][self.Routes[rand1][rand3]] + \
                               self.distance[self.Routes[rand1][rand3]][self.Routes[rand2][rand4 + 1]]
            else:
                new_route = copy.deepcopy(self.Routes)
                if rand3 > rand4:
                    left_right = (rand4, rand3)
                else:
                    left_right = (rand3, rand4)


                tmp = new_route[rand1][left_right[0]:left_right[1]+1]
                tmp.reverse()
                new_route[rand1][left_right[0]:left_right[1]+1]=tmp

                old_distance = self.distance[self.Routes[rand1][left_right[0] - 1]][self.Routes[rand1][left_right[0]]] + \
                               self.distance[self.Routes[rand1][left_right[1]]][self.Routes[rand1][left_right[1] + 1]]

                new_distance = self.distance[self.Routes[rand1][left_right[0] - 1]][self.Routes[rand1][left_right[1]]] + \
                               self.distance[self.Routes[rand1][left_right[0]]][self.Routes[rand1][left_right[1] + 1]]

            dis1 = new_distance - old_distance


            action = []
            cost = []
            for i in range(action_num):

                leagal_flag = 0
                while (leagal_flag == 0):
                    rand = np.random.randint(0,2)   

                    if rand==0:
                        rand1 = np.random.randint(0, len(new_route))
                        rand2 = np.random.randint(0, len(new_route))
                        rand3 = np.random.randint(1, len(new_route[rand1])-1)
                        rand4 = np.random.randint(1, len(new_route[rand2])-1)
                        capacity = 0

                        if rand1 != rand2:
                            for j in range(len(new_route[rand2])):
                                capacity+=self.demand[new_route[rand2][j]]

                            capacity+=self.demand[new_route[rand1][rand3]]
                            if capacity > self.tons:
                                continue

                    else :
                        rand1 = np.random.randint(0, len(new_route))
                        rand2 = rand1
                        rand3 = np.random.randint(1, len(new_route[rand1])-1)
                        rand4 = np.random.randint(1, len(new_route[rand2])-1)


                    if (rand1 == rand2 and abs(rand3 - rand4) <= 1) or (
                            new_route[rand1][rand3] in self.first_place_list) or (
                            new_route[rand2][rand4] in self.first_place_list) or len(new_route[rand1])==3 :
                        continue

                    leagal_flag = 1

                old_distance = self.distance[new_route[rand1][rand3 - 1]][new_route[rand1][rand3]] + \
                               self.distance[new_route[rand1][rand3]][new_route[rand1][rand3 + 1]] \
                               + self.distance[new_route[rand2][rand4 - 1]][new_route[rand2][rand4]]
                new_distance = self.distance[new_route[rand1][rand3 - 1]][new_route[rand1][rand3 + 1]] + \
                               self.distance[new_route[rand2][rand4 - 1]][new_route[rand1][rand3]] \
                               + self.distance[new_route[rand1][rand3]][new_route[rand2][rand4]]
                action.append((rand1, rand2, rand3, rand4))
                cost.append(dis1 + new_distance - old_distance)

            if min(cost) < 0 :

                    better_route = action[cost.index(min(cost))]

                    if better_route[0] == better_route[1]:
                        if better_route[2] > better_route[3]:
                            tmp = new_route[better_route[0]][better_route[2]]
                            del new_route[better_route[0]][better_route[2]]
                            new_route[better_route[1]].insert(better_route[3], tmp)
                        else:
                            tmp = new_route[better_route[0]][better_route[2]]
                            new_route[better_route[1]].insert(better_route[3], tmp)
                            del new_route[better_route[0]][better_route[2]]
                    else:
                        new_route[better_route[1]].insert(better_route[3],new_route[better_route[0]][better_route[2]])
                        del new_route[better_route[0]][better_route[2]]

                    self.Routes = new_route
                    return  1

            return 0


    def constrain_car_num(self):

        if len(self.Routes) > self.car_num:
            really_route = []
            for j in range(len(self.Routes)-1,-1,-1):
                 if self.Routes[j][1] in self.first_place_list:
                     really_route.append(self.Routes[j])
                     del self.Routes[j]

            cap_list = []
            for i in range(len(self.Routes)):
                cap = 0
                for j in range(len(self.Routes[i])):
                    cap += self.demand[self.Routes[i][j]]
                cap_list.append(cap)

            while(len(really_route)!=self.car_num):

                index = cap_list.index(max(cap_list))
                really_route.append(self.Routes[index])
                del cap_list[index]

            self.Routes = really_route

    def get_task_index_list(self):
        self.task_list = []
        for i in range(len(self.Routes)):
            each_route = []
            for j in range(len(self.Routes[i])):
                each_route.append(self.node_list[self.Routes[i][j]])
            self.task_list.append(each_route)
        #print(self.task_list)

    def get_route_index_list(self):
        self.robot_route_list = []
        for i in range(len(self.Routes)):
            if self.Routes[i][1] in self.first_place_list:
                each_route = [(0,0),self.node_list[self.Routes[i][1]]]
            else:
                each_route = copy.deepcopy(self.route_list[self.Routes[i][0]][self.Routes[i][1]])
            for j in range(len(self.Routes[i])-2):
                del each_route[len(each_route) - 1]
                each_route += self.route_list[self.Routes[i][j+1]][self.Routes[i][j+2]]
            self.robot_route_list.append(each_route)
        #print(self.robot_route_list)

    def give_robot_route(self):
        index_list_1 = []
        index_list_2 = []
        self.route_index = []
        a_list = {1,2,3}
        for i in range(len(self.Routes)):
            if self.Routes[i][1] in self.first_place_list:
                index = self.first_place_list.index(self.Routes[i][1])
                index_list_1.append(i)
                index_list_2.append(self.first_robot_index[index])
                a_list.remove(self.first_robot_index[index])

        for i in range(len(self.Routes)):
            if i in index_list_1:
                #print(i)
                #print(index_list_2[index_list_1.index(i)])
                #print()
                self.route_index.append(index_list_2[index_list_1.index(i)])
            else:
                new = a_list.pop()
                self.route_index.append(new)
                if self.robot_route_list[i][1] != (-4+2*self.route_index[i],-6.5):
                    if self.robot_route_list[i][1] not in self.task_list[i] :
                        self.robot_route_list[i][1] = (-4+2*self.route_index[i],-6.5)  #.insert(1,(-4+2*self.route_index[i],-6.5))
                    else:
                        self.robot_route_list[i].insert(1,(-4+2*self.route_index[i],-6.5))

        for i in range(len(self.Routes)):
            if self.robot_route_list[i][len(self.robot_route_list[i])-2] != (-4+2*self.route_index[i],-6.5):
                if self.robot_route_list[i][len(self.robot_route_list[i])-2] not in self.task_list[i]:
                    self.robot_route_list[i][len(self.robot_route_list[i])-2] = (-4 + 2 * self.route_index[i], -6.5)  # .insert(1,(-4+2*self.route_index[i],-6.5))
                else:
                    self.robot_route_list[i].insert(len(self.robot_route_list[i])-1,(-4+2*self.route_index[i],-6.5))

                self.robot_route_list[i][len(self.robot_route_list[i])-1] = (-4+2*self.route_index[i],0)
        #print(self.route_index)

    def cal_everything(self):
        self.savingsAlgorithms()
        random_search_time = 5
        local_search_time = 5
        #for i in range(random_search_time):
            #self.exchange_and_relocate(local_search_time)

        self.constrain_car_num()
        #self.printRoutes()
        self.get_task_index_list()
        self.get_route_index_list()
        self.give_robot_route()
        # print('dvrp routes:')
        # print(self.robot_route_list)
        # print('dvrp tasks:')
        # print(self.task_list)
        # print('dvrp robots:')
        # print(self.route_index)
        return self.robot_route_list,self.task_list,self.route_index

if __name__=='__main__':
    distance_matrix = np.array([[0.0, 12.0, 8.6, 12.0, 25.0, 13.0, 24.0, 17.0],
                       [12.0, 0.0, 7.9, 5.1, 13.0, 18.1, 12.0, 26.9],
                       [8.6, 7.9, 0.0, 6.5, 20.9, 19.5, 18.5, 19.0],
                       [12.0, 5.1, 6.5, 0.0, 15.0, 13.0, 12.0, 23.0],
                       [25.0, 13.0, 20.9, 15.0, 0.0, 23.0, 4.7, 33.0],
                       [13.0, 18.1, 19.5, 13.0, 23.0, 0.0, 19.0, 10.0],
                       [24.0, 12.0, 18.5, 12.0, 4.7, 19.0, 0.0, 29.0],
                       [17.0, 26.9, 19.0, 23.0, 33.0, 10.0, 29.0, 0.0]])

    demand = {(9, 6.5): 20, (-4.5, -1.5): 13, (4.5, 6.5): 22, (9, -0.5): 0, (-10.5, -1.5): 0, (4.5, -0.5): 15, (-7.5, -6.5): 0, (9, -12.5): 20, (0, 0): 23, (9, -6.5): 13, (0, -6.5): 60, (4.5, -12.5): 10, (0, 6.5): 21, (-4.5, 6.5): 23, (4.5, -6.5): 0, (0, -12.5): 0}
    ton = 100



    dvrp = DVRP(place_num=16,ton=ton)
    t=0
    while t<3:
        dvrp.input_demand_and_car_num(demand_list=demand,car_num=3,first_place= [(0, -6.5)],robot_index=[3])
        a,b,c = dvrp.cal_everything()
        print(a)
        print(b)
        print(c)
        print(dvrp.Routes)
        t+=1




#def point_generater(init,dest):












