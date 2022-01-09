import vpython as vp 
import itertools 
import random 
import numpy as np 
from math import *
import matplotlib.pyplot as plt


scene = vp.canvas()

floor = vp.box(pos=vp.vector(0, 0, 0), length=100, height=0.001, width=100, color=vp.color.blue)

v = 0
dt = 0.001 
mass = 0.1 
g = 9.81


# k1 = 1000


def getCOM(v):
    COM = (v[0].pos + v[1].pos + v[2].pos + v[3].pos + v[4].pos + v[5].pos + v[6].pos + v[7].pos)/ 16
    return COM
    

def mutation(r):
    ran = random.randint(0, 9)
    if ran > mutation_rate:
        x = random.randint(0, 27)
        y = random.randint(0, 27)
#         z = random.randint(0,27)
        r[x], r[y] = r[y], r[x]
#         r[z][0] = np.random.uniform(-0.2,0.2)
#         r[z][1] = np.random.uniform(-np.pi,np.pi)
        return r 
    else:
        return r


def Crossover(m, n): 
    M = []
    N = []
    for i in range(len(m)): 
        M.append(m[i]) 
        N.append(n[i])

    rand = random.randint(0, 20)
    index = list(range(rand, rand + 8))
    exchangem = []
    exchangen = []
    for i in range(len(index)): 
        exchangem.append(M[index[i]]) 
        exchangen.append(N[index[i]])
    for j in range(len(index)): 
        M[index[j]] = exchangen[j] 
        N[index[j]] = exchangem[j]
    return (M, N)

dis1 = 0 
good_dis = [] 
best_dis = [] 
mutation_rate = 7 
## Generate 10 parents 
pa1 = []
for i in range(10):
    pa = []
    for i in range(28):
        p = []
        b = np.random.uniform(-0.2, 0.2)
        c = np.random.uniform(-np.pi, np.pi) 
        k1 = np.random.uniform(1000, 6000) 
        p.append(b)
        p.append(c) 
        p.append(k1) 
        pa.append(p)
    pa1.append(pa)
#print('PA1',len(pa1))

dots = []
for a in range(100):
    for i in np.arange(0, 10, 2):
        child1, child2 = Crossover(pa1[i], pa1[i + 1])
        pa1.append(child1)
        pa1.append(child2)

    for i in range(10, 20):
        pa1[i] = mutation(pa1[i])


    total_dis = []
    for h in range(20):
        ballname = ['b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8']
        ballvectors = [vp.vector(0, 0, 0), vp.vector(0, 1, 0), vp.vector(0, 0, 1), vp.vector(1, 0, 0),
        vp.vector(1, 1, 0), vp.vector(0, 1, 1), vp.vector(1, 0, 1), vp.vector(1, 1, 1)] 


    OriginalCOM = (ballvectors[0] + ballvectors[1] + ballvectors[2] + ballvectors[3] +ballvectors[4] + ballvectors[5] + ballvectors[6] + ballvectors[7]) / 16

    springvecs = []
    for i in range(len(ballname)):
        ballname[i] = vp.sphere(pos=ballvectors[i], radius=0.1, color=vp.color.red, velocity=vp.vector(0, 0, 0))
    for z in itertools.combinations(ballvectors, 2):
        springvecs.append(z)
        
        
    spring = ['s1', 's2', 's3', 's4', 's5', 's6', 's7', 's8', 's9', 's10', 's11', 's12','s13', 's14', 's15', 's16','s17', 's18', 's19', 's20', 's21', 's22', 's23', 's24', 's25', 's26', 's27','s28']

    for i in range(28):
        position = springvecs[i][1] - springvecs[i][0]
        spring[i] = vp.cylinder(pos=springvecs[i][0], axis=position, length=vp.mag(position), 
            radius=.03,color=vp.color.white) 

    g_vector = vp.vector(0, 9.81, 0)
    for i in range(len(ballname)):
        ballname[i].velocity = vp.vector(0, 0, 0)

    F_c = vp.vector(0, 1000, 0)
    L0 = np.zeros((8, 8))
    for i in range(8):
        for j in range(8):
            if i == j: 
                L0[j][i] = 0 
            else:
                position = ballname[j].pos - ballname[i].pos
                L0[j][i] = vp.mag(position)
    t = 0.001
    c = 1
    w = 10 * np.pi 
    while True:
       L0rate = np.zeros((8, 8))
       L0rate[0][1] = L0[0][1] + pa1[h][0][0] * sin(w * t + pa1[h][0][1])
       L0rate[1][0] = L0[1][0] + pa1[h][0][0] * sin(w * t + pa1[h][0][1])
       L0rate[0][2] = L0[0][2] + pa1[h][1][0] * sin(w * t + pa1[h][1][1])
       L0rate[2][0] = L0[2][0] + pa1[h][1][0] * sin(w * t + pa1[h][1][1])
       L0rate[0][3] = L0[0][3] + pa1[h][2][0] * sin(w * t + pa1[h][2][1])
       L0rate[3][0] = L0[3][0] + pa1[h][2][0] * sin(w * t + pa1[h][2][1])
       L0rate[0][4] = L0[0][4] + pa1[h][3][0] * sin(w * t + pa1[h][3][1])
       L0rate[4][0] = L0[4][0] + pa1[h][3][0] * sin(w * t + pa1[h][3][1])
       L0rate[0][5] = L0[0][5] + pa1[h][4][0] * sin(w * t + pa1[h][4][1])
       L0rate[5][0] = L0[5][0] + pa1[h][4][0] * sin(w * t + pa1[h][4][1])
       L0rate[0][6] = L0[0][6] + pa1[h][5][0] * sin(w * t + pa1[h][5][1])
       L0rate[6][0] = L0[6][0] + pa1[h][5][0] * sin(w * t + pa1[h][5][1])
       L0rate[0][7] = L0[0][7] + pa1[h][6][0] * sin(w * t + pa1[h][6][1])
       L0rate[7][0] = L0[7][0] + pa1[h][6][0] * sin(w * t + pa1[h][6][1])
       L0rate[1][2] = L0[1][2] + pa1[h][7][0] * sin(w * t + pa1[h][7][1])
       L0rate[2][1] = L0[2][1] + pa1[h][7][0] * sin(w * t + pa1[h][7][1])
       L0rate[1][3] = L0[1][3] + pa1[h][8][0] * sin(w * t + pa1[h][8][1])
       L0rate[3][1] = L0[3][1] + pa1[h][8][0] * sin(w * t + pa1[h][8][1])
       L0rate[1][4] = L0[1][4] + pa1[h][9][0] * sin(w * t + pa1[h][9][1])
       L0rate[4][1] = L0[4][1] + pa1[h][9][0] * sin(w * t + pa1[h][9][1])
       L0rate[1][5] = L0[1][5] + pa1[h][10][0] * sin(w * t + pa1[h][10][1])
       L0rate[5][1] = L0[5][1] + pa1[h][10][0] * sin(w * t + pa1[h][10][1])
       L0rate[1][6] = L0[1][6] + pa1[h][11][0] * sin(w * t + pa1[h][11][1])
       L0rate[6][1] = L0[6][1] + pa1[h][11][0] * sin(w * t + pa1[h][11][1])
       L0rate[1][7] = L0[1][7] + pa1[h][12][0] * sin(w * t + pa1[h][12][1])
       L0rate[7][1] = L0[7][1] + pa1[h][12][0] * sin(w * t + pa1[h][12][1])
       L0rate[2][3] = L0[2][3] + pa1[h][13][0] * sin(w * t + pa1[h][13][1])
       L0rate[3][2] = L0[3][2] + pa1[h][13][0] * sin(w * t + pa1[h][13][1])
       L0rate[2][4] = L0[2][4] + pa1[h][14][0] * sin(w * t + pa1[h][14][1])
       L0rate[4][2] = L0[4][2] + pa1[h][14][0] * sin(w * t + pa1[h][14][1])
       L0rate[2][5] = L0[2][5] + pa1[h][15][0] * sin(w * t + pa1[h][15][1])
       L0rate[5][2] = L0[5][2] + pa1[h][15][0] * sin(w * t + pa1[h][15][1])
       L0rate[2][6] = L0[2][6] + pa1[h][16][0] * sin(w * t + pa1[h][16][1])
       L0rate[6][2] = L0[6][2] + pa1[h][16][0] * sin(w * t + pa1[h][16][1])
       L0rate[2][7] = L0[2][7] + pa1[h][17][0] * sin(w * t + pa1[h][17][1])
       L0rate[7][2] = L0[7][2] + pa1[h][17][0] * sin(w * t + pa1[h][17][1])
       L0rate[3][4] = L0[3][4] + pa1[h][18][0] * sin(w * t + pa1[h][18][1])
       L0rate[4][3] = L0[4][3] + pa1[h][18][0] * sin(w * t + pa1[h][18][1])
       L0rate[3][5] = L0[3][5] + pa1[h][19][0] * sin(w * t + pa1[h][19][1])
       L0rate[5][3] = L0[5][3] + pa1[h][19][0] * sin(w * t + pa1[h][19][1])
       L0rate[3][6] = L0[3][6] + pa1[h][20][0] * sin(w * t + pa1[h][20][1])
       L0rate[6][3] = L0[6][3] + pa1[h][20][0] * sin(w * t + pa1[h][20][1])
       L0rate[3][7] = L0[3][7] + pa1[h][21][0] * sin(w * t + pa1[h][21][1])
       L0rate[7][3] = L0[7][3] + pa1[h][21][0] * sin(w * t + pa1[h][21][1])
       L0rate[4][5] = L0[4][5] + pa1[h][22][0] * sin(w * t + pa1[h][22][1])
       L0rate[5][4] = L0[5][4] + pa1[h][22][0] * sin(w * t + pa1[h][22][1])
       L0rate[4][6] = L0[4][6] + pa1[h][23][0] * sin(w * t + pa1[h][23][1])
       L0rate[6][4] = L0[6][4] + pa1[h][23][0] * sin(w * t + pa1[h][23][1])
       L0rate[4][7] = L0[4][7] + pa1[h][24][0] * sin(w * t + pa1[h][24][1])
       L0rate[7][4] = L0[7][4] + pa1[h][24][0] * sin(w * t + pa1[h][24][1])
       L0rate[5][6] = L0[5][6] + pa1[h][25][0] * sin(w * t + pa1[h][25][1])
       L0rate[6][5] = L0[6][5] + pa1[h][25][0] * sin(w * t + pa1[h][25][1])
       L0rate[5][7] = L0[5][7] + pa1[h][26][0] * sin(w * t + pa1[h][26][1])
       L0rate[7][5] = L0[7][5] + pa1[h][26][0] * sin(w * t + pa1[h][26][1])
       L0rate[6][7] = L0[6][7] + pa1[h][27][0] * sin(w * t + pa1[h][27][1])
       L0rate[7][6] = L0[7][6] + pa1[h][27][0] * sin(w * t + pa1[h][27][1])
       ks = np.zeros((8, 8))
       ks[0][1] = pa1[h][0][2]
       ks[1][0] = pa1[h][0][2]
       ks[0][2] = pa1[h][1][2]
       ks[2][0] = pa1[h][1][2]
       ks[0][3] = pa1[h][2][2]
       ks[3][0] = pa1[h][2][2]
       ks[0][4] = pa1[h][3][2]
       ks[4][0] = pa1[h][3][2]
       ks[0][5] = pa1[h][4][2]
       ks[5][0] = pa1[h][4][2]
       ks[0][6] = pa1[h][5][2]
       ks[6][0] = pa1[h][5][2]
       ks[0][7] = pa1[h][6][2]
       ks[7][0] = pa1[h][6][2]
       ks[1][2] = pa1[h][7][2]
       ks[2][1] = pa1[h][7][2]
       ks[1][3] = pa1[h][8][2]
       ks[3][1] = pa1[h][8][2]
       ks[1][4] = pa1[h][9][2]
       ks[4][1] = pa1[h][9][2]
       ks[1][5] = pa1[h][10][2]
       ks[5][1] = pa1[h][10][2]
       ks[1][6] = pa1[h][11][2]
       ks[6][1] = pa1[h][11][2]
       ks[1][7] = pa1[h][12][2]
       ks[7][1] = pa1[h][12][2]
       ks[2][3] = pa1[h][13][2]
       ks[3][2] = pa1[h][13][2]
       ks[2][4] = pa1[h][14][2]
       ks[4][2] = pa1[h][14][2]
       ks[2][5] = pa1[h][15][2]
       ks[5][2] = pa1[h][15][2]
       ks[2][6] = pa1[h][16][2]
       ks[6][2] = pa1[h][16][2]
       ks[2][7] = pa1[h][17][2]
       ks[7][2] = pa1[h][17][2]
       ks[3][4] = pa1[h][18][2]
       ks[4][3] = pa1[h][18][2]
       ks[3][5] = pa1[h][19][2]
       ks[5][3] = pa1[h][19][2]
       ks[3][6] = pa1[h][20][2]
       ks[6][3] = pa1[h][20][2]
       ks[3][7] = pa1[h][21][2]
       ks[7][3] = pa1[h][21][2]
       ks[4][5] = pa1[h][22][2]
       ks[5][4] = pa1[h][22][2]
       ks[4][6] = pa1[h][23][2]
       ks[6][4] = pa1[h][23][2]
       ks[4][7] = pa1[h][24][2]
       ks[7][4] = pa1[h][24][2]
       ks[5][6] = pa1[h][25][2]
       ks[6][5] = pa1[h][25][2]
       ks[5][7] = pa1[h][26][2]
       ks[7][5] = pa1[h][26][2]
       ks[6][7] = pa1[h][27][2]
       ks[7][6] = pa1[h][27][2]  
       t += 0.001
       for i in range(8):
           ballvectors[i] = ballname[i].pos
       springvecs = []
       for z in itertools.combinations(ballvectors, 2):
           springvecs.append(z)
       for i in range(28):
           position = springvecs[i][1] - springvecs[i][0]
           spring[i].pos = springvecs[i][0]
           spring[i].axis = position
           spring[i].length = vp.mag(position)
       dampening = 1
          
            
       F_mat = np.zeros((8, 8))
       F_vec = []
       F_v = []
       a = np.array(np.zeros((8, 8)))
       for i in range(8):
           for k in range(8):
               if k == i:
                   L = 0
                   F_mat[i][k] = 0
                   F_vec.append(vp.vector(0, 0, 0))
               else:
                   L = vp.mag(ballname[k].pos - ballname[i].pos) - L0rate[k][i]
                 # E_s.append(1/2*k_sp*L**2)
                   F_mat[i][k] = L * ks[k][i]
                   pf0 = ballname[k].pos - ballname[i].pos
                 # a[i,k] = vp.norm(pf0)*L*k_sp
                   F_vec.append(vp.norm(pf0) * L * ks[k][i])
                # E_S.append(sum(E_s)/2)
       a= np.array(F_vec).reshape(8, 8)
       F = a.sum(axis=0)
       for i in range(8):
           F[i] = F[i] + g_vector * mass
           if ballname[i].pos.y < floor.pos.y:
               F_N = ((floor.pos.y - ballname[i].pos.y) ** 2) * 800
               F[i].y = F[i].y - F_N
               mu = 1
               F_st = mu * F_N
               F_horiz = (F[i].x ** 2 + F[i].z ** 2) ** 0.5
               v_xz= (ballname[i].velocity.x ** 2 + ballname[i].velocity.z ** 2) ** 0.5
               vx = ballname[i].velocity.x / v_xz
               vz = ballname[i].velocity.z / v_xz
               if F_st < F_horiz:
                   F[i].x += F_horiz * vx - F_N * vx
                   F[i].z += F_horiz * vz - F_N * vz
               else:
                    F[i].x = F_horiz * vx
                    F[i].z = F_horiz * vz
                    ballname[i].velocity.x = 0
                    ballname[i].velocity.z = 0
       for i in range(8):
             ballname[i].velocity -= (F[i] / mass * dt) * dampening
             ballname[i].pos += ballname[i].velocity * dt
       c+= 1
       if c == 2000:
           break
"""
    # Calculating COM
       COM = getCOM(ballname)
       dvec = COM - OriginalCOM
       dis = sqrt(dvec.x ** 2 + dvec.z ** 2)
     # print(dis)
       total_dis.append(dis)
    dis_index = np.argsort(total_dis)
    sorted_dis = []
    sorted_pa1 = []
    for i in range(10):
        sorted_dis.append(total_dis[dis_index[i]])
        sorted_pa1.append(pa1[dis_index[i]])
    good_dis = sorted_dis[-10:]
    dots.append(good_dis)
    print('GOODDIS', good_dis[-1])
    best_dis.append(good_dis[-1])
    pa1 = sorted_pa1[-10:]
    print('PA1END', len(pa1))
"""

