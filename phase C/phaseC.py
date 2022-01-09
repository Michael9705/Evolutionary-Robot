import vpython as vp
import itertools
import random
import numpy as np
from math import *
v = 0
dt = 0.01
mass = 0.1
g = 9.81

mutation_rate = 10
method = ['sine','cosine','tanh']

scence = vp.canvas()

floor = vp.box(pos = vp.vector(0,0,0), length = 100, height = 0.001, 
               width = 100, color = vp.color.red)

def getDis(v):
    Dis = 0
    for i in range(8):
        Dis += v[i].pos
    return Dis/8
    
def mutation(r):
    rand = random.randint(0,9)
    if rand > mutation_rate:
        x = random.randint(0, len(r) - 1)
        y = random.randint(0, len(r) - 1)
        r[x],r[y] = r[y],r[x]
        return r
    else:
        return r
    
    def crossOver(a,b):
        M = []
        N = []
        for i in range(a):
            M.append(a[i])
            N.append(b[i])
        rand = random.randint(0, len(a) - 10)
        index = list(range(rand,rand + 10))
        
        l1 = []
        l2 = []
        
        for i in range(len(index)):
            l1.append(M[index[i]])
            l2.append(N[index[i]])
        for i in range(len(index)):
            M[index[i]] = l2[i]
            N[index[i]] = l1[i]
            
        return (M,N)
    
    
dis_0 = 0
dis_top = []
dis_down = []

rand = random.randint(0, 5)


parent = []
for i in range(10):
    parent_tmp = []
    for j in range(28):
        pa = []
        c1 = np.random.uniform(-0.2, 0.2)
        c2 = np.random.uniform(-np.pi, np.pi)
        c3 = np,random.uniform(1000,5000)
        pa.append(c1)
        pa.append(c2)
        pa.append(c3)
        parent_tmp.append(pa)
    parent.append(parent_tmp)
    
    
for i in range(100):
    for j in np.arange(0,10,2):
        child1,child2 = crossOver(parent[j], parent[j+1])
        parent.append(chile1)
        parent.append(child2)
    for j in range(10,20):
        parent[j] = mutation(parent[j])
        
massName = ['b1','b2','b3','b4','b5','b6','b7','b8']
massVetcor = [vp.vector(0,0,0),vp.vector(0,1,0),vp.vector(0,0,1),
             vp.vector(1,0,0),vp.vector(1,1,0),
             vp.vector(0,1,1),vp.vector(1,0,1),vp.vector(1,1,1)]

Dis = 0
for i in range(8):
    Dis += massVector[i]
Dis = Dis/16

springVector = []
for i in range(len(massName)):
    massName[i] = vp.sphere(pos = massVetcor[i],radius = 0.1, color = vp.color.blue)
    
vel = vp.vector(0,0,0)

for i in itertools.combinations(massVetcor,i):
    springVector.append(i)
    
    
springName = ['s1','s2','s3','s4','s5','s6','s7','s8','s9',
              's10','s11','s12','s13','s14','s15','s16','s17',
              's18','s19','s20','s21','s22','s23','s24','s25',
              's26','s27','s28']

for i in range(len(springName)):
    pos = springVector[i][1] - springVector[i][0]
    springName[i] = vp.cylinder(pos = springVector[i][0], axis = pos, length = vp.mag(pos),
                                radius = 0.3, color = vp.color.white)
    
for i in range(len(massName)):
    massName[i].vel = vp.vector(0,0,0)
    
gVector = (0,g,0)



def generateMass(param):
    if param ==0:
        
        ### mass_1
        massName_1 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_1 = [vp.vector(1,0,0),vp.vector(1,1,0),vp.vector(1,0,1),
                        vp.vector(1,1,1),vp.vector(2,0,0),vp.vector(2,1,0),
                        vp.vector(2,0,1),vp.vector(2,1,1)]
        Dis_1 = (massVector_1[0]+massVector_1[1]+massVector_1[2]+massVector_1[3]+massVector_1[4]+
                 massVector_1[5]+massVector_1[6]+massVector_1[6]+massVector_1[7]) / 8
        springVector_1 = []
        for i in range(len(massName_1)):
            massName_1[i] = vp.sphere(pos=massVector_1[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_1,2):
            springVector_1.append(z)
        spring_1 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_1 = springVecotr_1[i][1] - springVector_1[i][0]
            spring_1[1] = vp.cylinder(pos=springVector_1[i][0],axis = position_1,length=vp.mag(position_1),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_1)):
            massName_1[i].velocity = vp.vector(0,0,0)
            
        ### mass_2
        massName_2 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_2 = [vp.vector(0,0,1),vp.vector(0,1,1),vp.vector(1,0,1),
                        vp.vector(1,1,1),vp.vector(0,0,2),vp.vector(0,1,2),
                        vp.vector(1,0,2),vp.vector(1,1,2)]
        Dis_2 = (massVector_2[0]+massVector_2[1]+massVector_2[2]+massVector_2[3]+massVector_2[4]+
                 massVector_2[5]+massVector_2[6]+massVector_2[6]+massVector_2[7]) / 8
        springVector_2 = []
        for i in range(len(massName_2)):
            massName_2[i] = vp.sphere(pos=massVector_2[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_2,2):
            springVector_2.append(z)
        spring_2 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_2 = springVecotr_2[i][1] - springVector_2[i][0]
            spring_2[1] = vp.cylinder(pos=springVector_2[i][0],axis = position_2,length=vp.mag(position_2),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_2)):
            massName_2[i].velocity = vp.vector(0,0,0)
            
        ### mass_3
        massName_3 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_3 = [vp.vector(1,0,1),vp.vector(1,1,1),vp.vector(2,1,1),
                        vp.vector(2,0,1),vp.vector(1,1,2),vp.vector(1,0,2),
                        vp.vector(2,1,2),vp.vector(2,0,2)]
        Dis_3 = (massVector_3[0]+massVector_3[1]+massVector_3[2]+massVector_3[3]+massVector_3[4]+
                 massVector_3[5]+massVector_3[6]+massVector_3[6]+massVector_3[7]) / 8
        springVector_3 = []
        for i in range(len(massName_3)):
            massName_3[i] = vp.sphere(pos=massVector_3[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_3,2):
            springVector_3.append(z)
        spring_3 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_3 = springVecotr_3[i][1] - springVector_3[i][0]
            spring_3[1] = vp.cylinder(pos=springVector_3[i][0],axis = position_3,length=vp.mag(position_3),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_3)):
            massName_3[i].velocity = vp.vector(0,0,0)
            
        ### mass_4
        massName_4 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_4 = [vp.vector(0,1,0),vp.vector(1,1,0),vp.vector(0,1,1),
                        vp.vector(1,1,1),vp.vector(0,2,0),vp.vector(1,2,0),
                        vp.vector(0,1,2),vp.vector(1,2,1)]
        Dis_4 = (massVector_4[0]+massVector_4[1]+massVector_4[2]+massVector_4[3]+massVector_4[4]+
                 massVector_4[5]+massVector_4[6]+massVector_4[6]+massVector_4[7]) / 8
        springVector_4 = []
        for i in range(len(massName_4)):
            massName_4[i] = vp.sphere(pos=massVector_4[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_4,2):
            springVector_4.append(z)
        spring_4 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_4 = springVecotr_4[i][1] - springVector_4[i][0]
            spring_4[1] = vp.cylinder(pos=springVector_4[i][0],axis = position_4,length=vp.mag(position_4),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_4)):
            massName_4[i].velocity = vp.vector(0,0,0)
            
            
        ### mass_5
        massName_5 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_5 = [vp.vector(1,1,1),vp.vector(1,2,1),vp.vector(2,1,1),
                        vp.vector(2,2,1),vp.vector(1,1,2),vp.vector(1,2,2),
                        vp.vector(2,1,2),vp.vector(2,2,2)]
        Dis_5 = (massVector_5[0]+massVector_5[1]+massVector_5[2]+massVector_5[3]+massVector_5[4]+
                 massVector_5[5]+massVector_5[6]+massVector_5[6]+massVector_5[7]) / 8
        springVector_5 = []
        for i in range(len(massName_2)):
            massName_5[i] = vp.sphere(pos=massVector_5[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_5,2):
            springVector_5.append(z)
        spring_5 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_5 = springVecotr_5[i][1] - springVector_5[i][0]
            spring_5[1] = vp.cylinder(pos=springVector_5[i][0],axis = position_5,length=vp.mag(position_5),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_5)):
            massName_5[i].velocity = vp.vector(0,0,0)
        
        
        ### mass_6
        massName_6 = ['b11','b22','b33','b44','b55','b66','b77','b88']
        massVector_6 = [vp.vector(1,1,0),vp.vector(1,1,1),vp.vector(1,2,0),
                        vp.vector(1,2,1),vp.vector(2,1,2),vp.vector(2,2,2),
                        vp.vector(2,2,0),vp.vector(2,2,1)]
        Dis_6 = (massVector_6[0]+massVector_6[1]+massVector_6[2]+massVector_6[3]+massVector_6[4]+
                 massVector_6[5]+massVector_6[6]+massVector_6[6]+massVector_6[7]) / 8
        springVector_6 = []
        for i in range(len(massName_6)):
            massName_6[i] = vp.sphere(pos=massVector_6[i],radius = 0.1, color =vp.color.blue,velocity = (0,0,0))
        for j in itertools.combinations(massVector_6,2):
            springVector_6.append(z)
        spring_6 = ['s1','s2','s3','s4','s5','s6','s7',
                    's8','s9','s10','s11','s12','s13','s14',
                    's15','s16','s17','s18','s19','s20','s21','s22','s23','s24','s25','s26','s27','s28',]
        for i in range (28):
            position_6 = springVecotr_6[i][1] - springVector_6[i][0]
            spring_6[1] = vp.cylinder(pos=springVector_6[i][0],axis = position_6,length=vp.mag(position_6),radius=0.03,
                                    color = vp.color.white)
        for i in range(len(massName_6)):
            massName_6[i].velocity = vp.vector(0,0,0)
            
        TotalDis = Dis_1 + Dis_2 + Dis_3 + Dis_4 + Dis_5 + Dis_6
        
        
        
        