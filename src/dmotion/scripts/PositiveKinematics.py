import numpy as np
import math as m
from numpy import linalg


def getLeftT(i, a):
    t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0
    if i == 1:
        t1 = a
    elif i == 2:
        t2 = a
    elif i == 3:
        t3 = a
    elif i == 4:
        t4 = a
    elif i == 5:
        t5 = a
    elif i == 6:
        t6 = a
    elif i == 7:
        t7 = 0
    C = np.array([[0, 4.5, -8, -m.pi/2+t1],
        [m.pi/2, 0, 0, m.pi/2+t2],
        [m.pi/2, 0, 0, t3],
        [m.pi/2, -12, 0, t4],
        [0, -12, 0, -t5],
        [-m.pi/2, 0, 0, -t6],
        [0, -3.5, 0, t7]])    
    T = np.array([[m.cos(C[i-1][3]), -m.sin(C[i-1][3]), 0, C[i-1][1]],
         [m.sin(C[i-1][3])*m.cos(C[i-1][0]), m.cos(C[i-1][3])*m.cos(C[i-1][0]), -m.sin(C[i-1][0]), -m.sin(C[i-1][0])*C[i-1][2]],
         [m.sin(C[i-1][3])*m.sin(C[i-1][0]), m.cos(C[i-1][3])*m.sin(C[i-1][0]), m.cos(C[i-1][0]), m.cos(C[i-1][0])*C[i-1][2]],
         [0, 0, 0, 1]])
    return T


def getRightT(i, a):
    t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0
    if i == 1:
        t1 = a
    elif i == 2:
        t2 = a
    elif i == 3:
        t3 = a
    elif i == 4:
        t4 = a
    elif i == 5:
        t5 = a
    elif i == 6:
        t6 = a
    elif i == 7:
        t7 = 0
    C = np.array([[0, -4.5, -8, -m.pi/2-t1],
        [m.pi/2, 0, 0, m.pi/2+t2],
        [m.pi/2, 0, 0, -t3],
        [m.pi/2, -12, 0, t4],
        [0, -12, 0, -t5],
        [-m.pi/2, 0, 0, t6],
        [0, -3.5, 0, t7]])    
    T = np.array([[m.cos(C[i-1][3]), -m.sin(C[i-1][3]), 0, C[i-1][1]],
         [m.sin(C[i-1][3])*m.cos(C[i-1][0]), m.cos(C[i-1][3])*m.cos(C[i-1][0]), -m.sin(C[i-1][0]), -m.sin(C[i-1][0])*C[i-1][2]],
         [m.sin(C[i-1][3])*m.sin(C[i-1][0]), m.cos(C[i-1][3])*m.sin(C[i-1][0]), m.cos(C[i-1][0]), m.cos(C[i-1][0])*C[i-1][2]],
         [0, 0, 0, 1]])
    return T


file1 = open('/home/jingxin/climb.txt', 'r')
a = file1.readlines()
f = []
for i in a:
    g = [float(j) for j in i.split()]
    f.append(g)
# b = [float(j) for j in input().split()]
print(f)
file1.close()
final = []
for i in range(len(f)):
    b = f[i][0:12]
    time = f[i][16]
    c = []
    for i in b:
        c.append(i / 180 * m.pi)
    right = c[0:6]
    left = c[6:12]
    right_hip_roll = right[1]
    right_hip_pitch = right[2]
    left_hip_roll = left[1]
    left_hip_pitch = left[2]
    right[1] = right_hip_pitch
    right[2] = right_hip_roll
    left[1] = left_hip_pitch
    left[2] = left_hip_roll
    left.append(0)
    right.append(0)

    TLC = getLeftT(1, left[0])
    for i in range(1, 7):
        TLC = TLC.dot(getLeftT(i+1, left[i]))
    T87 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    TLC = TLC.dot(T87)

    TRC = getRightT(1, right[0])
    for i in range(1, 7):
        TRC = TRC.dot(getRightT(i+1, right[i]))
    TRC = TRC.dot(T87)

    TCR = linalg.inv(TRC)
    TLR = TCR.dot(TLC)
    TRL = linalg.inv(TLR)

    RLC_init = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    RLC_init_1 = linalg.inv(RLC_init)
    RLC = TLC[0:3, 0:3]
    RLL_init = RLC_init_1.dot(RLC)

    RRC_init = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    RRC_init_1 = linalg.inv(RRC_init)
    RRC = TRC[0:3, 0:3]
    RRR_init = RRC_init_1.dot(RRC)

    RLR_init = TLR[0:3, 0:3]
    RRL_init = TRL[0:3, 0:3]

    LCz = TLC[2][3]
    LCx = -TLC[1][3]
    LCy = TLC[0][3]

    RCz = TRC[2][3]
    RCx = -TRC[1][3]
    RCy = TRC[0][3]

    LRz = TLR[2][3]
    LRx = TLR[0][3]
    LRy = TLR[1][3]

    RLz = TRL[2][3]
    RLx = TRL[0][3]
    RLy = TRL[1][3]

    pitch_LR = -m.asin(RLR_init[2][0])
    roll_LR = m.asin(RLR_init[2][1] / m.cos(pitch_LR))
    # print(RLR_init[0][0] / m.cos(pitch))
    yaw_LR = m.acos(RLR_init[0][0] / m.cos(pitch_LR))

    pitch_RL = -m.asin(RRL_init[2][0])
    roll_RL = m.asin(RRL_init[2][1] / m.cos(pitch_RL))
    # print(RRL_init[0][0] / m.cos(pitch))
    yaw_RL = m.acos(RRL_init[0][0] / m.cos(pitch_RL))

    pitch_left = -m.asin(RLL_init[2][0])
    roll_left = m.asin(RLL_init[2][1] / m.cos(pitch_left))
    # print(RLL_init[0][0] / m.cos(pitch))
    yaw_left = m.acos(RLL_init[0][0] / m.cos(pitch_left))

    pitch_right = -m.asin(RRR_init[2][0])
    roll_right = m.asin(RRR_init[2][1] / m.cos(pitch_right))
    # print(RRR_init[0][0] / m.cos(pitch))
    yaw_right = m.acos(RRR_init[0][0] / m.cos(pitch_right))

    pitch_left = pitch_left / m.pi * 180
    roll_left = roll_left / m.pi * 180
    yaw_left = yaw_left / m.pi * 180
    pitch_right = pitch_right / m.pi * 180
    roll_right = roll_right / m.pi * 180
    yaw_right = yaw_right / m.pi * 180

    pitch_LR = pitch_LR / m.pi * 180
    roll_LR = roll_LR / m.pi * 180
    yaw_LR = yaw_LR / m.pi * 180
    pitch_RL = pitch_RL / m.pi * 180
    roll_RL = roll_RL / m.pi * 180
    yaw_RL = yaw_RL / m.pi * 180

    final.append(RLx)
    final.append(RLy)
    final.append(RLz)
    final.append(roll_RL)
    final.append(pitch_RL)
    final.append(yaw_RL)
    final.append(time)
    print(final)
    file2 = open('/home/jingxin/position.txt', 'a')
    for j in range(len(final)):
        file2.write(str(final[j]))
        file2.write(' ')
    file2.write('\n')
    final.clear()
    file2.close()
    print("左腿相对于中心坐标系的x y z:")
    print(LCx)
    print(LCy)
    print(LCz)
    print("左腿相对于中心坐标系的x y z:")
    print(RCx)
    print(RCy)
    print(RCz)
    print("左腿相对于右腿的x y z:")
    print(LRx)
    print(LRy)
    print(LRz)
    print("右腿相对于左腿的x y z:")
    print(RLx)
    print(RLy)
    print(RLz)
    print("左腿相对于左腿标准坐标系的r p y:")
    print(roll_left)
    print(pitch_left)
    print(yaw_left)
    print("右腿相对于右腿标准坐标系的r p y:")
    print(roll_right)
    print(pitch_right)
    print(yaw_right)
    print("左腿相对于右腿的r p y:")
    print(roll_LR)
    print(pitch_LR)
    print(yaw_LR)
    print("右腿相对于左腿的r p y:")
    print(roll_RL)
    print(pitch_RL)
    print(yaw_RL)