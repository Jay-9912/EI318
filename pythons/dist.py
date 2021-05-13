import numpy as np

node_list=[(9,6.5),(4.5,6.5),(0,6.5),(-4.5,6.5),(-4.5,-1.5),(-10.5,-1.5),\
           (-7.5,-6.5),(0,-6.5),(0,-12.5),(4.5,-12.5),(4.5,-6.5),(9,-12.5),\
           (9,-6.5),(9,-0.5),(4.5,-0.5)] # 任务地点

neighbor={} # 邻接表
neighbor[0]=[1,13]
neighbor[1]=[0,2,14]
neighbor[2]=[1,3]
neighbor[3]=[4,5]
neighbor[4]=[3,5,6,7]
neighbor[5]=[3,4,6]
neighbor[6]=[4,5,7]
neighbor[7]=[4,6,8,10]
neighbor[8]=[7,9]
neighbor[9]=[8,10,11]
neighbor[10]=[7,8,9,12,14]
neighbor[11]=[9,12]
neighbor[12]=[10,11]
neighbor[13]=[0,14]
neighbor[14]=[1,10,13]

def manh_dist(s1,s2): # 曼哈顿距离
    return abs(s1[0]-s2[0])+abs(s1[1]-s2[1])

def floyd(d): # 弗洛伊德算法
    D=d
    lengthD = len(D)                    #邻接矩阵大小
    p = list(range(lengthD))
    P = []
    for i in range(lengthD):
        P.append(p)
    P = np.array(P)
    for k in range(lengthD):
        for i in range(lengthD):
            for j in range(lengthD):
                if(D[i][j] >D[i][k]+D[k][j]):         #两个顶点直接较小的间接路径替换较大的直接路径
                    P[i][j] = P[i][k]                 #记录新路径的前驱
                    D[i][j] = D[i][k]+D[k][j]
    print('各个顶点的最短路径:')
    for i in range(lengthD):
        for j in range(i+1,lengthD):
            print('v%d' % (i+1) + '--' + 'v%d' % (j+1) + '\t' + 'dist_min:' + '\t' + str(D[i][j]) + '\t' + 'path:'+'v%d'%(i+1),end='' )
            temp=P[i][j]
            while (temp!=j):
                print('--'+'v%d'%(temp+1),end='')
                temp=P[temp][j]
            print('--'+'v%d'%(j+1))
    print('P矩阵:')
    print(P)
    print('D矩阵:')
    print(D)
    return D

if __name__=="__main__":
    d=np.zeros((15,15))
    for i in range(15):
        for j in range(15):
            if i==j:
                d[i][j]=0
            if j in neighbor[i]:
                d[i][j]=manh_dist(node_list[i],node_list[j])
            else:
                d[i][j]=1000
    print(d)
    Dist=floyd(d)
